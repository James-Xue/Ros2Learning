// main.cpp
// ROS 2 系统信息 Qt 查看器主入口

#include <memory>

#include <QApplication>
#include <QFont>
#include <QMainWindow>
#include <QTextEdit>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include "ros2_learning_sysinfo_qt_viewer/sysinfo_qt_node.hpp"

/**
 * @brief 主函数
 * 
 * 设计特点：
 * - 将 ROS 2 的 spin_some() 嵌入 Qt 事件循环
 * - 不额外创建线程，避免 UI 线程/ROS 线程切换的同步复杂度
 * - 通过 QTimer 周期调用 executor.spin_some()，处理订阅回调
 * 
 * 工作流程：
 * 1. 初始化 Qt 应用和 ROS 2
 * 2. 创建 Qt 主窗口和文本显示控件
 * 3. 创建 ROS 2 节点并添加到 executor
 * 4. 使用 QTimer 定期触发 ROS 2 消息处理
 * 5. 进入 Qt 主事件循环
 */
int main(int argc, char** argv)
{
    // Qt 会解析并移除它认识的参数；剩余参数仍可供 rclcpp 使用
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    // ═══════════════════════════════════════════════════════
    // 创建 Qt 主窗口
    // ═══════════════════════════════════════════════════════
    QMainWindow window;
    window.setWindowTitle("ROS2 SysInfo Viewer (Qt)");

    auto* text = new QTextEdit();
    text->setReadOnly(true);
    
    // 使用等宽字体，更适合"日志/JSON/表格"的显示
    QFont mono;
    mono.setStyleHint(QFont::Monospace);
    mono.setFamily("Monospace");
    text->setFont(mono);
    
    window.setCentralWidget(text);
    window.resize(900, 600);
    window.show();

    // ═══════════════════════════════════════════════════════
    // 创建 ROS 2 节点和 executor
    // ═══════════════════════════════════════════════════════
    using ros2_learning_sysinfo_qt_viewer::SysInfoQtNode;
    auto node = std::make_shared<SysInfoQtNode>(text);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // ═══════════════════════════════════════════════════════
    // 集成 ROS 2 到 Qt 事件循环
    // ═══════════════════════════════════════════════════════
    // 以较高频率处理 ROS 回调；UI 更新频率取决于发布端 publish_rate_hz
    QTimer ros_spin_timer;
    QObject::connect(&ros_spin_timer, &QTimer::timeout,
                     [&exec]() { exec.spin_some(); });
    ros_spin_timer.start(10);  // 每 10ms 处理一次 ROS 消息

    // ═══════════════════════════════════════════════════════
    // 启动 Qt 主循环
    // ═══════════════════════════════════════════════════════
    const int ret = app.exec();

    // 清理
    exec.remove_node(node);
    rclcpp::shutdown();
    return ret;
}
