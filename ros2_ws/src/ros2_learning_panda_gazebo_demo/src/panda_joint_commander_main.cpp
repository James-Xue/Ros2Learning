#include "ros2_learning_panda_gazebo_demo/panda_joint_commander.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    // 1. 初始化 ROS 2 通信层
    // 这必须在创建任何节点之前调用，用于处理命令行参数（如重映射）。
    rclcpp::init(argc, argv);

    // 2. 创建节点实例
    // 使用 shared_ptr 管理节点生命周期是 ROS 2 的推荐做法。
    auto node = std::make_shared<ros2_learning_panda_gazebo_demo::PandaJointCommander>();

    // 3. 进入事件循环 (Spin)
    // rclcpp::spin(node) 会阻塞主线程，并开始处理回调函数（如我们的 onTimer）。
    // 直到节点被请求关闭（例如按下 Ctrl+C）。
    rclcpp::spin(node);

    // 4. 关闭 ROS 2
    // 释放资源，清理通信上下文。
    rclcpp::shutdown();
    return 0;
}
