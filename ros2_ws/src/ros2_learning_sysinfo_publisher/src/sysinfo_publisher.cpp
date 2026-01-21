#include <chrono>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <sys/utsname.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{
std::string trim(const std::string &s)
{
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::optional<std::string> read_first_line(const std::string &path)
{
    std::ifstream in(path);
    if (!in.is_open())
    {
        return std::nullopt;
    }
    std::string line;
    if (!std::getline(in, line))
    {
        return std::nullopt;
    }
    return trim(line);
}

std::string json_escape(const std::string &s)
{
    std::string out;
    out.reserve(s.size() + 8);
    for (const unsigned char c : s)
    {
        switch (c)
        {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\b':
            out += "\\b";
            break;
        case '\f':
            out += "\\f";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (c < 0x20)
            {
                char buf[7];
                std::snprintf(buf, sizeof(buf), "\\u%04x",
                              static_cast<unsigned>(c));
                out += buf;
            }
            else
            {
                out += static_cast<char>(c);
            }
            break;
        }
    }
    return out;
}

std::unordered_map<std::string, std::string>
parse_kv_file(const std::string &path)
{
    std::unordered_map<std::string, std::string> kv;
    std::ifstream in(path);
    if (!in.is_open())
    {
        return kv;
    }

    std::string line;
    while (std::getline(in, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string::npos)
        {
            continue;
        }
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"')
        {
            val = val.substr(1, val.size() - 2);
        }
        kv[key] = val;
    }
    return kv;
}

struct MemInfo
{
    long mem_total_kb{0};
    long mem_available_kb{0};
};

MemInfo read_meminfo()
{
    MemInfo info;
    std::ifstream in("/proc/meminfo");
    if (!in.is_open())
    {
        return info;
    }

    std::string key;
    long value = 0;
    std::string unit;
    while (in >> key >> value >> unit)
    {
        if (key == "MemTotal:")
        {
            info.mem_total_kb = value;
        }
        else if (key == "MemAvailable:")
        {
            info.mem_available_kb = value;
        }
    }
    return info;
}

struct CpuInfo
{
    std::string model_name;
    int logical_cores{0};
    std::string loadavg;
};

CpuInfo read_cpuinfo()
{
    CpuInfo cpu;

    // model name + logical core count
    {
        std::ifstream in("/proc/cpuinfo");
        if (in.is_open())
        {
            std::string line;
            while (std::getline(in, line))
            {
                if (line.rfind("model name", 0) == 0)
                {
                    const auto pos = line.find(':');
                    if (pos != std::string::npos && cpu.model_name.empty())
                    {
                        cpu.model_name = trim(line.substr(pos + 1));
                    }
                    cpu.logical_cores += 1;
                }
            }
        }
    }

    // loadavg
    {
        auto maybe = read_first_line("/proc/loadavg");
        if (maybe)
        {
            cpu.loadavg = *maybe;
        }
    }

    return cpu;
}

struct NetDev
{
    std::string iface;
    unsigned long long rx_bytes{0};
    unsigned long long tx_bytes{0};
};

std::vector<NetDev> read_netdev()
{
    std::vector<NetDev> out;
    std::ifstream in("/proc/net/dev");
    if (!in.is_open())
    {
        return out;
    }

    // skip 2 header lines
    std::string line;
    std::getline(in, line);
    std::getline(in, line);

    while (std::getline(in, line))
    {
        // format: iface: rx_bytes ... tx_bytes ...
        const auto colon = line.find(':');
        if (colon == std::string::npos)
        {
            continue;
        }
        std::string iface = trim(line.substr(0, colon));
        if (iface.empty())
        {
            continue;
        }

        std::istringstream iss(line.substr(colon + 1));
        unsigned long long rx = 0;
        unsigned long long tx = 0;
        // rx_bytes is first number
        iss >> rx;
        // skip next 7 rx fields
        for (int i = 0; i < 7; ++i)
        {
            unsigned long long dummy = 0;
            iss >> dummy;
        }
        // tx_bytes is next
        iss >> tx;

        // skip loopback to reduce noise
        if (iface != "lo")
        {
            out.push_back(NetDev{iface, rx, tx});
        }
    }

    return out;
}

std::string read_uname_string()
{
    utsname u{};
    if (uname(&u) != 0)
    {
        return {};
    }
    std::ostringstream oss;
    oss << u.sysname << " " << u.release << " " << u.version << " "
        << u.machine;
    return oss.str();
}

std::string read_hostname()
{
    char buf[256];
    if (gethostname(buf, sizeof(buf)) != 0)
    {
        return {};
    }
    buf[sizeof(buf) - 1] = '\0';
    return std::string(buf);
}

std::string read_os_pretty_name()
{
    const auto kv = parse_kv_file("/etc/os-release");
    const auto it = kv.find("PRETTY_NAME");
    if (it != kv.end())
    {
        return it->second;
    }
    return {};
}

std::string read_uptime_seconds()
{
    auto maybe = read_first_line("/proc/uptime");
    if (!maybe)
    {
        return {};
    }
    // uptime file: "<uptime_seconds> <idle_seconds>"
    std::istringstream iss(*maybe);
    double uptime_s = 0.0;
    iss >> uptime_s;
    std::ostringstream oss;
    oss << uptime_s;
    return oss.str();
}

} // namespace

class SysInfoPublisherNode : public rclcpp::Node
{
  public:
    SysInfoPublisherNode()
            : rclcpp::Node("ros2_learning_sysinfo_publisher")
    {
        topic_ = this->declare_parameter<std::string>("topic",
                                                      "/ros2_learning/sysinfo");
        publish_rate_hz_ =
            this->declare_parameter<double>("publish_rate_hz", 1.0);
        if (publish_rate_hz_ <= 0.0)
        {
            publish_rate_hz_ = 1.0;
        }

        pub_ = this->create_publisher<std_msgs::msg::String>(topic_, 10);

        const auto period =
            std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SysInfoPublisherNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "Publishing sysinfo to %s at %.2f Hz",
                    topic_.c_str(), publish_rate_hz_);
    }

  private:
    void on_timer()
    {
        // 设计意图：发布一条“自描述的文本”，方便用 `ros2 topic echo` 直接查看。
        // 这里选用 JSON（字符串）是为了让后续 UI 或脚本解析更容易。
        const std::string hostname = read_hostname();
        const std::string os_name = read_os_pretty_name();
        const std::string kernel = read_uname_string();
        const std::string uptime_s = read_uptime_seconds();

        const CpuInfo cpu = read_cpuinfo();
        const MemInfo mem = read_meminfo();
        const auto net = read_netdev();

        std::ostringstream oss;
        oss << "{";
        oss << "\"hostname\":\"" << json_escape(hostname) << "\",";
        oss << "\"os\":\"" << json_escape(os_name) << "\",";
        oss << "\"kernel\":\"" << json_escape(kernel) << "\",";
        oss << "\"uptime_s\":" << (uptime_s.empty() ? "null" : uptime_s) << ",";

        oss << "\"cpu\":{";
        oss << "\"model\":\"" << json_escape(cpu.model_name) << "\",";
        oss << "\"logical_cores\":" << cpu.logical_cores << ",";
        oss << "\"loadavg\":\"" << json_escape(cpu.loadavg) << "\"";
        oss << "},";

        oss << "\"memory_kb\":{";
        oss << "\"total\":" << mem.mem_total_kb << ",";
        oss << "\"available\":" << mem.mem_available_kb;
        oss << "},";

        oss << "\"net\":[";
        for (size_t i = 0; i < net.size(); ++i)
        {
            const auto &n = net[i];
            if (i != 0)
            {
                oss << ",";
            }
            oss << "{";
            oss << "\"iface\":\"" << json_escape(n.iface) << "\",";
            oss << "\"rx_bytes\":" << n.rx_bytes << ",";
            oss << "\"tx_bytes\":" << n.tx_bytes;
            oss << "}";
        }
        oss << "]";

        oss << "}";

        std_msgs::msg::String msg;
        msg.data = oss.str();
        pub_->publish(msg);
    }

    std::string topic_;
    double publish_rate_hz_{1.0};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SysInfoPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
