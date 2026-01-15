#pragma once

// 这个头文件只放 TeleopKeyboardNode 的“声明”。
// 目的：
// - 让节点类接口更清晰（声明/实现分离）
// - 便于 IDE / 调试器索引符号（尤其是 Debug 构建下的跳转与断点）
//
// 注意：键盘输入/终端 raw mode 的实现细节被刻意隐藏在 .cpp 内部，
// 这里通过 forward-declare + unique_ptr 来避免把 termios
// 等平台相关头文件暴露给外部。

#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_learning_turtlebot3_teleop::detail
{
class TerminalRawMode;
} // namespace ros2_learning_turtlebot3_teleop::detail

// 键盘遥控节点：读取键盘输入并持续发布 TwistStamped 到 cmd_vel。
// - 默认发布频率由参数 publish_rate_hz 控制
// - 当超过 command_timeout_s 未收到新输入时，会自动发布 stop
//
// 重要设计点：
// - 节点采用“固定频率发布 + 超时刹停”，而不是“有按键才发布”。
//   这样下游控制器更容易处理，也能避免键盘输入中断导致持续运动。
// - raw-mode 与 termios
// 等平台相关细节不放在这里，避免头文件膨胀、减少编译耦合。
class TeleopKeyboardNode : public rclcpp::Node
{
  public:
    // 构造函数会声明并读取参数、创建 publisher/timer，并尝试切换终端到 raw
    // mode。
    TeleopKeyboardNode();

    // unique_ptr 持有一个 incomplete type，析构必须在 .cpp
    // 里定义（保证类型完整）。
    ~TeleopKeyboardNode() override;

  private:
    // 定时回调：处理键盘输入并按固定频率发布指令。
    void on_timer();

    // 读取键盘输入并更新 current_cmd_ 与缩放因子。
    void process_keyboard();

    // 平台相关的终端 raw-mode 控制器，放在 .cpp 中实现。
    std::unique_ptr<ros2_learning_turtlebot3_teleop::detail::TerminalRawMode>
        raw_;

    // cmd_vel_topic：发布的目标 topic（默认 /cmd_vel）。
    std::string cmd_vel_topic_;
    // 这些默认值会在构造函数初始化列表中赋值；
    // 同时也会作为 declare_parameter() 的默认值，保持行为一致。
    // linear/angular_scale：按键对应的线速度/角速度缩放。
    // 注意：这些值会在运行过程中被按键（q/e/r/f/t/g）动态调整。
    double linear_scale_;
    double angular_scale_;

    // publish_rate_hz：定时器发布频率。
    double publish_rate_hz_;

    // command_timeout_s：超过该时长未收到新输入则自动 stop。
    double command_timeout_s_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // current_cmd_：当前“要发布”的速度命令（header.stamp 在发布前填充）。
    geometry_msgs::msg::TwistStamped current_cmd_;

    // last_cmd_time_：上次“命令或缩放更新”的时间点，用于超时逻辑。
    rclcpp::Time last_cmd_time_;
};
