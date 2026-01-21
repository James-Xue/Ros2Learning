// system_reader.hpp
// 系统信息读取工具

#pragma once

#include <string>
#include <vector>

namespace ros2_learning_sysinfo_publisher
{

/**
 * @brief CPU 信息结构
 */
struct CpuInfo
{
    std::string model_name;   ///< CPU 型号
    int logical_cores{0};     ///< 逻辑核心数
    std::string loadavg;      ///< 负载平均值
};

/**
 * @brief 内存信息结构
 */
struct MemInfo
{
    long mem_total_kb{0};      ///< 总内存 (kB)
    long mem_available_kb{0};  ///< 可用内存 (kB)
};

/**
 * @brief 网络接口信息结构
 */
struct NetDev
{
    std::string iface;              ///< 接口名称
    unsigned long long rx_bytes{0}; ///< 接收字节数
    unsigned long long tx_bytes{0}; ///< 发送字节数
};

/**
 * @brief 读取主机名
 */
std::string readHostname();

/**
 * @brief 读取操作系统名称（从 /etc/os-release）
 */
std::string readOsPrettyName();

/**
 * @brief 读取内核信息（uname）
 */
std::string readUnameString();

/**
 * @brief 读取系统运行时间（秒）
 */
std::string readUptimeSeconds();

/**
 * @brief 读取 CPU 信息
 */
CpuInfo readCpuInfo();

/**
 * @brief 读取内存信息
 */
MemInfo readMemInfo();

/**
 * @brief 读取网络接口信息
 */
std::vector<NetDev> readNetDev();

/**
 * @brief JSON 字符串转义
 */
std::string jsonEscape(const std::string& s);

} // namespace ros2_learning_sysinfo_publisher
