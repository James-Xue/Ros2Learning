#include "ros2_learning_panda_gazebo_demo/panda_joint_commander.hpp"

#include <cmath>

namespace ros2_learning_panda_gazebo_demo
{

// 构造函数
PandaJointCommander::PandaJointCommander()
: Node("panda_joint_commander"), start_time_(this->now())
{
    // 定义 7 个关节的控制话题名称。
    // 这些话题名称必须与 gazebo_ros2_control 或 bridge 中定义的一致。
    // 在本例中，这些话题接收 std_msgs::msg::Float64 类型的消息来控制关节位置。
    const std::array<std::string, 7> topics = {
        "/panda/joint1/cmd_pos",
        "/panda/joint2/cmd_pos",
        "/panda/joint3/cmd_pos",
        "/panda/joint4/cmd_pos",
        "/panda/joint5/cmd_pos",
        "/panda/joint6/cmd_pos",
        "/panda/joint7/cmd_pos",
    };

    // 预分配内存，避免 vector 扩容带来的开销
    publishers_.reserve(topics.size());
    
    // 循环创建每个关节的发布者
    for (const auto & topic : topics) {
        // 创建发布者，消息类型为 Float64，队列长度为 10
        publishers_.push_back(this->create_publisher<std_msgs::msg::Float64>(topic, 10));
    }

    // 创建一个壁钟定时器 (Wall Timer)，周期为 50 毫秒 (20 Hz)。
    // 这意味着 onTimer 函数每秒钟会被调用 20 次，从而产生连贯的控制信号。
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PandaJointCommander::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Panda joint commander started.");
}

// 定时器回调函数：核心控制逻辑
void PandaJointCommander::onTimer()
{
    // 1. 计算当前运行时间 t (从节点启动开始计算的秒数)
    // 这个时间 t 将作为正弦函数的自变量，驱动机械臂进行周期性运动。
    const double t = (this->now() - start_time_).seconds();

    // 2. 定义每个关节的运动参数
    
    // 相位偏移 (Phase Offset)：
    // 让每个关节的运动在时间上错开，而不是所有关节同时达到最大值或最小值。
    // 这样可以让机械臂的整体动作看起来更像是在“舞动”或“波动”，更加自然。
    const std::array<double, 7> phase = {0.0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8};

    // 关节软限位 (Soft Limits)：
    // Panda 机械臂的物理关节极限。我们在代码中硬编码这些值是为了安全，
    // 防止发出的命令让机械臂试图撞击自身的物理硬限位。
    const std::array<double, 7> lower = {-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671};
    const std::array<double, 7> upper = { 2.9671,  1.8326,  2.9671,  0.0873,  2.9671,  3.8223,  2.9671};

    // 3. 计算并发布每个关节的命令
    for (size_t i = 0; i < publishers_.size(); ++i) {
        // 计算运动范围的中心点
        const double mid = (lower[i] + upper[i]) * 0.5;
        
        // 计算运动幅度：我们只使用关节总范围的 30% (0.3) 进行演示，确保运动幅度适中，安全可靠。
        const double amplitude = (upper[i] - lower[i]) * 0.3;
        
        std_msgs::msg::Float64 cmd;
        
        // 核心运动公式：位置 = 中心点 + 振幅 * sin(时间 + 相位)
        // 这是一个标准的正弦波发生器。
        cmd.data = mid + amplitude * std::sin(t + phase[i]);
        
        // 安全保护：最后一道防线
        // 确保计算出的命令绝对不会超出关节的上限和下限。
        if (cmd.data > upper[i]) {
            cmd.data = upper[i];
        } else if (cmd.data < lower[i]) {
            cmd.data = lower[i];
        }
        
        // 发布命令到 ROS topic
        publishers_[i]->publish(cmd);
    }
}

}  // namespace ros2_learning_panda_gazebo_demo
