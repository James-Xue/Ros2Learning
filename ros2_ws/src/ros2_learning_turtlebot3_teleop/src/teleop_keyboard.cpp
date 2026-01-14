// teleop_keyboard.cpp
//
// 代码意图（非常重要，方便学习与调试）：
// 1) 本节点通过“定时器”以固定频率发布 TwistStamped（而不是按键触发才发布）。
//    - 好处：下游控制器能稳定接收控制指令；调试时也更容易观察到周期性行为。
// 2) 键盘读取采用“非阻塞”方式：每次定时器回调里尽可能把当前缓冲区的按键读完。
//    - 好处：不会卡住 executor；即使没有按键也能继续按频率发布/超时停机。
// 3) 为安全起见，引入 command_timeout_s：超过该时间未更新指令则自动发布 stop。
//    - 好处：避免终端/网络/调试暂停等导致机器人持续运动。
//
// 文件职责划分：
// - TeleopKeyboardNode 的“声明”在 include/ 里（便于 IDE/调试器索引符号）。
// - 终端 raw-mode 的平台细节被隔离在 terminal_raw_mode.*（避免 termios
// 头文件污染）。
// - 本文件保留：键盘读取/帮助文本/节点方法实现/main。

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../include/teleop_keyboard_node.hpp"
#include "../include/terminal_raw_mode.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

using namespace std::chrono_literals;

// 为了让头文件更“干净”，所有终端 raw mode / 键盘读取的细节都放到 detail
// 命名空间里。 外部只需要知道 TeleopKeyboardNode。
namespace ros2_learning_turtlebot3_teleop::detail
{
int read_key_nonblocking()
{
    // 这里用 select() 只做“探测”：STDIN 是否有数据可读。
    // - 这样不会阻塞线程（tv=0），避免 timer callback 卡住。
    // - 返回 -1 表示当前没有按键，调用方可以直接退出循环。
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    // select 返回：>0 可读；=0 超时（这里等价于“无按键”）；<0 出错。
    const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0)
    {
        return -1;
    }

    char c = 0;
    // 只读 1 个字节：我们只关心单字符按键。
    // 在 raw mode 下，read 能直接返回按键，不必等待回车。
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0)
    {
        return -1;
    }
    return static_cast<unsigned char>(c);
}

std::string help_text()
{
    // 说明：help_text() 返回字符串而不是直接 RCLCPP_INFO，
    // 这样调用方可以灵活选择输出位置（启动时/按 ? 时）。
    return std::string("Keyboard Teleop (C++)\n"
                       "-------------------\n"
                       "w/s : forward/back\n"
                       "a/d : turn left/right\n"
                       "x or space : stop\n"
                       "q/e : increase/decrease both speeds\n"
                       "r/f : increase/decrease linear speed\n"
                       "t/g : increase/decrease angular speed\n"
                       "?   : show this help\n"
                       "CTRL-C to quit\n");
}

} // namespace ros2_learning_turtlebot3_teleop::detail

TeleopKeyboardNode::TeleopKeyboardNode()
        : rclcpp::Node("ros2_learning_teleop_keyboard")
        , raw_(std::make_unique<
               ros2_learning_turtlebot3_teleop::detail::TerminalRawMode>())
        , cmd_vel_topic_("/cmd_vel")
        , linear_scale_(0.22)
        , angular_scale_(2.84)
        , publish_rate_hz_(20.0)
        , command_timeout_s_(0.5)
        , current_cmd_()
{
    // 参数声明：这些默认值尽量选取 turtlebot3 常用配置，便于开箱即用。
    // 注意：成员变量也会先初始化为这些值，然后再通过 get_parameter 覆盖。
    // 这样做的意图是：即使参数系统异常，也能保持一个“合理默认”。
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("linear_scale", 0.22);
    this->declare_parameter<double>("angular_scale", 2.84);
    this->declare_parameter<double>("publish_rate_hz", 20.0);
    this->declare_parameter<double>("command_timeout_s", 0.5);

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    command_timeout_s_ = this->get_parameter("command_timeout_s").as_double();

    // 防御性：publish_rate_hz_ 必须为正；否则定时周期会无意义/崩溃。
    if (publish_rate_hz_ <= 0.0)
    {
        publish_rate_hz_ = 20.0;
    }

    // Publisher：queue depth 10 对键盘遥控已足够。
    // 如果下游消费慢，旧消息会被丢弃，通常符合“只要最新速度”这一语义。
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "%s",
                ros2_learning_turtlebot3_teleop::detail::help_text().c_str());
    RCLCPP_INFO(this->get_logger(),
                "Publishing to %s (linear_scale=%.3f angular_scale=%.3f)",
                cmd_vel_topic_.c_str(), linear_scale_, angular_scale_);

    // enable() 失败通常是：不是在 TTY 中运行、或权限/termios 调用失败。
    // 失败时仍然继续运行（只是键盘输入可能不可用），方便在 launch/日志里排查。
    // 进入 raw mode：让按键不必回车即可读取。
    // 注意：在非交互 TTY（例如某些 launch/重定向）下可能失败。
    if (!raw_ || !raw_->enable())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to enable terminal raw mode (is this a TTY?). "
                    "Keyboard input may not work.");
    }

    // 记录“最后一次收到有效输入/更新”时间。
    // 用于超时逻辑：超过 command_timeout_s 没有新输入时自动 stop。
    last_cmd_time_ = this->now();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&TeleopKeyboardNode::on_timer, this));
}

TeleopKeyboardNode::~TeleopKeyboardNode() = default;

void TeleopKeyboardNode::on_timer()
{
    // 定时器回调：
    // 1) 先尽可能处理当前缓冲区按键（可能一次读到多个）。
    // 2) 再根据超时与当前指令发布 TwistStamped。
    process_keyboard();

    const auto now = this->now();
    const double age = (now - last_cmd_time_).seconds();
    // 超时：发布 stop（Twist 默认全 0）。
    // 设计点：即使之前有速度指令，超时也要刹停，避免机器人“跑飞”。
    if (age > command_timeout_s_)
    {
        geometry_msgs::msg::TwistStamped stop;
        stop.header.stamp = now;
        pub_->publish(stop);
        return;
    }

    current_cmd_.header.stamp = now;
    pub_->publish(current_cmd_);
}

void TeleopKeyboardNode::process_keyboard()
{
    // 注意这里是“循环读键”，直到 read_key_nonblocking() 告诉我们“没键可读”。
    // 这样可以一次性处理掉用户快速输入的多次按键，避免积压到下一次 timer。
    for (;;)
    {
        const int key =
            ros2_learning_turtlebot3_teleop::detail::read_key_nonblocking();
        if (key < 0)
        {
            break;
        }

        bool updated_cmd = false;
        bool updated_scale = false;

        // 这里的映射遵循常见的键盘遥控习惯：WASD + X/space stop。
        // updated_cmd 表示“速度命令更新”；updated_scale 表示“缩放因子更新”。
        switch (key)
        {
        case 'w':
        case 'W':
            current_cmd_.twist.linear.x = linear_scale_;
            current_cmd_.twist.angular.z = 0.0;
            updated_cmd = true;
            break;
        case 's':
        case 'S':
            current_cmd_.twist.linear.x = -linear_scale_;
            current_cmd_.twist.angular.z = 0.0;
            updated_cmd = true;
            break;
        case 'a':
        case 'A':
            current_cmd_.twist.linear.x = 0.0;
            current_cmd_.twist.angular.z = angular_scale_;
            updated_cmd = true;
            break;
        case 'd':
        case 'D':
            current_cmd_.twist.linear.x = 0.0;
            current_cmd_.twist.angular.z = -angular_scale_;
            updated_cmd = true;
            break;
        case 'x':
        case 'X':
        case ' ':
            // stop：直接清零 TwistStamped。
            // 注意：header 会在 on_timer() 中填充当前时间戳。
            current_cmd_ = geometry_msgs::msg::TwistStamped{};
            updated_cmd = true;
            break;
        case 'q':
        case 'Q':
            linear_scale_ *= 1.1;
            angular_scale_ *= 1.1;
            updated_scale = true;
            break;
        case 'e':
        case 'E':
            linear_scale_ *= 0.9;
            angular_scale_ *= 0.9;
            updated_scale = true;
            break;
        case 'r':
        case 'R':
            linear_scale_ *= 1.1;
            updated_scale = true;
            break;
        case 'f':
        case 'F':
            linear_scale_ *= 0.9;
            updated_scale = true;
            break;
        case 't':
        case 'T':
            angular_scale_ *= 1.1;
            updated_scale = true;
            break;
        case 'g':
        case 'G':
            angular_scale_ *= 0.9;
            updated_scale = true;
            break;
        case '?':
            // 帮助：输出按键说明，方便在调试终端里随时查看。
            RCLCPP_INFO(
                this->get_logger(), "%s",
                ros2_learning_turtlebot3_teleop::detail::help_text().c_str());
            break;
        default:
            break;
        }

        // 缩放更新后做一次夹紧，避免出现负数。
        if (updated_scale)
        {
            if (linear_scale_ < 0.0)
            {
                linear_scale_ = 0.0;
            }
            if (angular_scale_ < 0.0)
            {
                angular_scale_ = 0.0;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Updated scales: linear_scale=%.3f angular_scale=%.3f",
                        linear_scale_, angular_scale_);
        }

        // 只要命令或缩放发生变化，就认为“收到新输入”。
        // 这样可以避免一直触发超时 stop。
        if (updated_cmd || updated_scale)
        {
            last_cmd_time_ = this->now();
        }
    }
}
