// manipulation_stub 节点入口：初始化 ROS2 并保持自旋
#include "ros2_learning_manipulation_stub/manipulation_stub.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    try
    {
        // 创建节点并自旋处理服务请求
        auto node = std::make_shared<ManipulationStub>();
        rclcpp::spin(node);
        // 正常退出
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        // 捕获异常，避免静默退出
        fprintf(stderr, "[manipulation_stub] FATAL: exception: %s\n", e.what());
        rclcpp::shutdown();
        return 2;
    }
}
