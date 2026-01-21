// sysinfo_qt_node.hpp
// ROS 2 节点类，集成 Qt 界面订阅系统信息

#pragma once

#include <memory>
#include <string>

#include <QTextEdit>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros2_learning_sysinfo_qt_viewer
{

/**
 * @brief 系统信息 Qt 查看器节点
 * 
 * 功能：
 * - 订阅系统信息话题（JSON 格式）
 * - 解析并格式化数据
 * - 在 Qt TextEdit 控件中显示
 * 
 * 注意：
 * - 设计为在 Qt 主线程中运行
 * - 通过 QTimer 定期调用 spin_some() 处理回调
 * - 回调可以直接更新 UI，无需信号/槽机制
 */
class SysInfoQtNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param view Qt 文本显示控件指针（非所有权）
     */
    explicit SysInfoQtNode(QTextEdit* view);

    /**
     * @brief 析构函数
     */
    ~SysInfoQtNode() override = default;

private:
    std::string m_topic;  ///< 订阅的话题名称
    QTextEdit* m_view;    ///< Qt 文本显示控件（非所有权）
    
    /// ROS 2 订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub;
};

} // namespace ros2_learning_sysinfo_qt_viewer
