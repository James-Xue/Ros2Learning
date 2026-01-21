// system_reader.cpp
// 系统信息读取工具实现

#include "ros2_learning_sysinfo_publisher/system_reader.hpp"

#include <cctype>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <unordered_map>

#include <sys/utsname.h>
#include <unistd.h>

namespace ros2_learning_sysinfo_publisher
{

namespace
{

std::string trim(const std::string& s)
{
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::optional<std::string> readFirstLine(const std::string& path)
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

std::unordered_map<std::string, std::string>
parseKvFile(const std::string& path)
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

} // namespace

std::string jsonEscape(const std::string& s)
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

std::string readHostname()
{
    char buf[256];
    if (gethostname(buf, sizeof(buf)) != 0)
    {
        return {};
    }
    buf[sizeof(buf) - 1] = '\0';
    return std::string(buf);
}

std::string readOsPrettyName()
{
    const auto kv = parseKvFile("/etc/os-release");
    const auto it = kv.find("PRETTY_NAME");
    if (it != kv.end())
    {
        return it->second;
    }
    return {};
}

std::string readUnameString()
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

std::string readUptimeSeconds()
{
    auto maybe = readFirstLine("/proc/uptime");
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

CpuInfo readCpuInfo()
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
        auto maybe = readFirstLine("/proc/loadavg");
        if (maybe)
        {
            cpu.loadavg = *maybe;
        }
    }

    return cpu;
}

MemInfo readMemInfo()
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

std::vector<NetDev> readNetDev()
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

} // namespace ros2_learning_sysinfo_publisher
