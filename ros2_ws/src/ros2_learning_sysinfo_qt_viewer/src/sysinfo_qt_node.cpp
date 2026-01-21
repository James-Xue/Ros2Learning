// sysinfo_qt_node.cpp
// ROS 2 节点类实现

#include "ros2_learning_sysinfo_qt_viewer/sysinfo_qt_node.hpp"
#include "ros2_learning_sysinfo_qt_viewer/json_formatter.hpp"

#include <QString>

namespace ros2_learning_sysinfo_qt_viewer
{

SysInfoQtNode::SysInfoQtNode(QTextEdit* view)
    : rclcpp::Node("ros2_learning_sysinfo_qt_viewer")
    , m_view(view)
{
    // 声明话题参数，允许用户自定义订阅的话题
    m_topic = this->declare_parameter<std::string>("topic",
                                                   "/ros2_learning/sysinfo");

    // 创建订阅器
    m_sub = this->create_subscription<std_msgs::msg::String>(
        m_topic, 10,
        [this](const std_msgs::msg::String& msg)
        {
            // 回调运行在 Qt 主线程（因为我们在 Qt 的 timer 里 spin_some）。
            // 所以可以直接更新 UI。
            if (m_view)
            {
                const QString raw = QString::fromStdString(msg.data);
                m_view->setPlainText(formatSystemInfo(raw));
            }
        });

    // 显示初始状态
    if (m_view)
    {
        m_view->setPlainText(
            QString("Subscribed to %1\nWaiting for messages...")
                .arg(QString::fromStdString(m_topic)));
    }
}

} // namespace ros2_learning_sysinfo_qt_viewer
