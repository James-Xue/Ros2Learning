// 引入智能指针支持
#include <memory>

// 包含本地实现的 Nav2Client 类头文件（定义在 nav2_client.hpp）
#include "ros2_learning_nav2_client/nav2_client.hpp"
// rclcpp 核心头文件，提供 ROS 2 节点初始化、shutdown、日志等功能
#include "rclcpp/rclcpp.hpp"

// 程序入口：初始化 ROS 2、创建并运行 Nav2Client 节点，退出前进行清理
int main(int argc, char **argv)
{
    // 初始化 rclcpp，必须在任何 ROS 2 API 调用前执行，传入命令行参数
    rclcpp::init(argc, argv);

    try
    {
        // 使用智能指针创建 Nav2Client 节点实例（在构造函数中会创建 publisher 与
        // action client）
        auto node = std::make_shared<Nav2Client>();

        // 调用节点的运行入口函数，内部会执行等待时间、发布初始位姿、等待
        // TF、发送目标并等待结果
        node->run();

        // 清理并关闭 rclcpp，释放资源
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        // 避免异常未捕获导致进程直接 SIGABRT（表现为 "Aborted"）
        fprintf(stderr, "[nav2_client] FATAL: exception: %s\n", e.what());
        rclcpp::shutdown();
        return 2;
    }
}
