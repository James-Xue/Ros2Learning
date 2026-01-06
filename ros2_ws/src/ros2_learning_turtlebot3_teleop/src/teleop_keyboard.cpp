// teleop_keyboard.cpp
// - 这个文件保留“实现”部分（键盘读取、raw mode、节点方法实现、main）。
// - 节点类的“声明”已抽到 include/ 下，便于工程组织与调试跳转。

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_learning_turtlebot3_teleop/teleop_keyboard_node.hpp"
#include "ros2_learning_turtlebot3_teleop/terminal_raw_mode.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

using namespace std::chrono_literals;

// 为了让头文件更“干净”，所有终端 raw mode / 键盘读取的细节都放到 detail 命名空间里。
// 外部只需要知道 TeleopKeyboardNode。
namespace ros2_learning_turtlebot3_teleop::detail
{
int read_key_nonblocking()
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0)
    {
        return -1;
    }

    char c = 0;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n <= 0)
    {
        return -1;
    }
    return static_cast<unsigned char>(c);
}

std::string help_text()
{
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
        , raw_(std::make_unique<ros2_learning_turtlebot3_teleop::detail::TerminalRawMode>())
        , cmd_vel_topic_("/cmd_vel")
        , linear_scale_(0.22)
        , angular_scale_(2.84)
        , publish_rate_hz_(20.0)
        , command_timeout_s_(0.5)
        , current_cmd_()
{
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

    if (publish_rate_hz_ <= 0.0)
    {
        publish_rate_hz_ = 20.0;
    }

    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "%s", ros2_learning_turtlebot3_teleop::detail::help_text().c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to %s (linear_scale=%.3f angular_scale=%.3f)", cmd_vel_topic_.c_str(),
                linear_scale_, angular_scale_);

    // enable() 失败通常是：不是在 TTY 中运行、或权限/termios 调用失败。
    // 失败时仍然继续运行（只是键盘输入可能不可用），方便在 launch/日志里排查。
    if (!raw_ || !raw_->enable())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to enable terminal raw mode (is this a TTY?). Keyboard input may not work.");
    }

    last_cmd_time_ = this->now();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&TeleopKeyboardNode::on_timer, this));
}

TeleopKeyboardNode::~TeleopKeyboardNode() = default;

void TeleopKeyboardNode::on_timer()
{
    process_keyboard();

    const auto now = this->now();
    const double age = (now - last_cmd_time_).seconds();
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
    for (;;)
    {
        const int key = ros2_learning_turtlebot3_teleop::detail::read_key_nonblocking();
        if (key < 0)
        {
            break;
        }

        bool updated_cmd = false;
        bool updated_scale = false;

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
            RCLCPP_INFO(this->get_logger(), "%s", ros2_learning_turtlebot3_teleop::detail::help_text().c_str());
            break;
        default:
            break;
        }

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
            RCLCPP_INFO(this->get_logger(), "Updated scales: linear_scale=%.3f angular_scale=%.3f", linear_scale_,
                        angular_scale_);
        }

        if (updated_cmd || updated_scale)
        {
            last_cmd_time_ = this->now();
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
