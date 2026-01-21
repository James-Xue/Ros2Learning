// sysinfo_publisher_node.cpp
// 系统信息发布器节点实现

#include "ros2_learning_sysinfo_publisher/sysinfo_publisher_node.hpp"
#include "ros2_learning_sysinfo_publisher/system_reader.hpp"

#include <sstream>

using namespace std::chrono_literals;

namespace ros2_learning_sysinfo_publisher
{

SysInfoPublisherNode::SysInfoPublisherNode()
    : rclcpp::Node("ros2_learning_sysinfo_publisher")
{
    // 声明参数
    m_topic = this->declare_parameter<std::string>("topic",
                                                   "/ros2_learning/sysinfo");
    m_publishRateHz = this->declare_parameter<double>("publish_rate_hz", 1.0);
    
    if (m_publishRateHz <= 0.0)
    {
        m_publishRateHz = 1.0;
    }

    // 创建发布器
    m_pub = this->create_publisher<std_msgs::msg::String>(m_topic, 10);

    // 创建定时器
    const auto period = std::chrono::duration<double>(1.0 / m_publishRateHz);
    m_timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&SysInfoPublisherNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Publishing sysinfo to %s at %.2f Hz",
                m_topic.c_str(), m_publishRateHz);
}

void SysInfoPublisherNode::onTimer()
{
    // 设计意图：发布一条"自描述的文本"，方便用 `ros2 topic echo` 直接查看。
    // 这里选用 JSON（字符串）是为了让后续 UI 或脚本解析更容易。
    
    // 读取系统信息
    const std::string hostname = readHostname();
    const std::string os_name = readOsPrettyName();
    const std::string kernel = readUnameString();
    const std::string uptime_s = readUptimeSeconds();

    const CpuInfo cpu = readCpuInfo();
    const MemInfo mem = readMemInfo();
    const auto net = readNetDev();

    // 构建 JSON 字符串
    std::ostringstream oss;
    oss << "{";
    oss << "\"hostname\":\"" << jsonEscape(hostname) << "\",";
    oss << "\"os\":\"" << jsonEscape(os_name) << "\",";
    oss << "\"kernel\":\"" << jsonEscape(kernel) << "\",";
    oss << "\"uptime_s\":" << (uptime_s.empty() ? "null" : uptime_s) << ",";

    oss << "\"cpu\":{";
    oss << "\"model\":\"" << jsonEscape(cpu.model_name) << "\",";
    oss << "\"logical_cores\":" << cpu.logical_cores << ",";
    oss << "\"loadavg\":\"" << jsonEscape(cpu.loadavg) << "\"";
    oss << "},";

    oss << "\"memory_kb\":{";
    oss << "\"total\":" << mem.mem_total_kb << ",";
    oss << "\"available\":" << mem.mem_available_kb;
    oss << "},";

    oss << "\"net\":[";
    for (size_t i = 0; i < net.size(); ++i)
    {
        const auto& n = net[i];
        if (i != 0)
        {
            oss << ",";
        }
        oss << "{";
        oss << "\"iface\":\"" << jsonEscape(n.iface) << "\",";
        oss << "\"rx_bytes\":" << n.rx_bytes << ",";
        oss << "\"tx_bytes\":" << n.tx_bytes;
        oss << "}";
    }
    oss << "]";

    oss << "}";

    // 发布消息
    std_msgs::msg::String msg;
    msg.data = oss.str();
    m_pub->publish(msg);
}

} // namespace ros2_learning_sysinfo_publisher
