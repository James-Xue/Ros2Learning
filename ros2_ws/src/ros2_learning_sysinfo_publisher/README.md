# ROS2 SysInfo Publisher

一个 ROS 2 系统信息发布器，定期采集 Linux 系统信息（CPU、内存、网络等）并以 JSON 格式发布。

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![C++17](https://img.shields.io/badge/C++-17-orange)

---

## 📋 功能特性

- ✅ **实时系统监控**: 定期采集系统运行状态
- ✅ **多维度信息**: CPU、内存、网络、OS、内核版本等
- ✅ **JSON 格式**: 易于解析和可视化
- ✅ **可配置**: 支持自定义话题和发布频率
- ✅ **轻量级**: 直接读取 `/proc` 文件系统，无需外部依赖

---

## 🏗️ 架构设计

### 项目结构

```
ros2_learning_sysinfo_publisher/
├── include/ros2_learning_sysinfo_publisher/
│   ├── system_reader.hpp          # 系统信息读取工具
│   └── sysinfo_publisher_node.hpp # 发布器节点类
├── src/
│   ├── system_reader.cpp          # 系统读取实现
│   ├── sysinfo_publisher_node.cpp # 节点实现
│   └── main.cpp                   # 主入口
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 数据来源

| 信息类型 | Linux 数据源 |
|---------|------------|
| CPU 型号/核心数 | `/proc/cpuinfo` |
| CPU 负载 | `/proc/loadavg` |
| 内存 | `/proc/meminfo` |
| 网络流量 | `/proc/net/dev` |
| 运行时间 | `/proc/uptime` |
| 操作系统 | `/etc/os-release` |
| 内核版本 | `uname()` |
| 主机名 | `gethostname()` |

---

## 🔧 依赖项

### ROS 2 依赖
- `rclcpp` - ROS 2 C++ 客户端库
- `std_msgs` - 标准消息类型

### 系统依赖
- Linux 系统（需要 `/proc` 文件系统）

---

## 🚀 构建和安装

```bash
# 1. 进入工作空间
cd ~/ros2_ws

# 2. 编译包
colcon build --packages-select ros2_learning_sysinfo_publisher

# 3. source 环境
source install/setup.bash
```

---

## 📖 使用方法

###基本用法

```bash
# 默认配置（1Hz，话题 /ros2_learning/sysinfo）
ros2 run ros2_learning_sysinfo_publisher sysinfo_publisher
```

### 自定义发布频率

```bash
# 每秒发布 2 次
ros2 run ros2_learning_sysinfo_publisher sysinfo_publisher \
  --ros-args -p publish_rate_hz:=2.0
```

### 自定义话题名称

```bash
ros2 run ros2_learning_sysinfo_publisher sysinfo_publisher \
  --ros-args -p topic:=/custom/sysinfo
```

### 查看发布的消息

```bash
# 查看话题列表
ros2 topic list | grep sysinfo

# 实时查看发布的 JSON 数据
ros2 topic echo /ros2_learning/sysinfo
```

---

## 📊 JSON 数据格式

发布的消息示例：

```json
{
  "hostname": "wen",
  "os": "Ubuntu 22.04.3 LTS",
  "kernel": "Linux 5.15.0-91-generic #101-Ubuntu SMP x86_64",
  "uptime_s": 123456.78,
  "cpu": {
    "model": "Intel(R) Core(TM) i7-9750H CPU @ 2.60GHz",
    "logical_cores": 12,
    "loadavg": "1.23 2.34 3.45"
  },
  "memory_kb": {
    "total": 16384000,
    "available": 8192000
  },
  "net": [
    {
      "iface": "eth0",
      "rx_bytes": 123456789,
      "tx_bytes": 987654321
    },
    {
      "iface": "wlan0",
      "rx_bytes": 12345,
      "tx_bytes": 54321
    }
  ]
}
```

### 字段说明

| 字段 | 类型 | 说明 |
|------|------|------|
| `hostname` | String | 主机名 |
| `os` | String | 操作系统名称和版本 |
| `kernel` | String | 内核版本信息 |
| `uptime_s` | Number | 系统运行时间（秒） |
| `cpu.model` | String | CPU 型号 |
| `cpu.logical_cores` | Number | CPU 逻辑核心数 |
| `cpu.loadavg` | String | 1/5/15 分钟负载平均值 |
| `memory_kb.total` | Number | 总内存（kB） |
| `memory_kb.available` | Number | 可用内存（kB） |
| `net[].iface` | String | 网络接口名称 |
| `net[].rx_bytes` | Number | 接收字节数 |
| `net[].tx_bytes` | Number | 发送字节数 |

---

## 🎨 技术细节

### 模块设计

#### 1. **SystemReader** - 系统信息读取模块
- **职责**: 封装所有系统信息读取逻辑
- **特点**: 无 ROS 依赖，可独立测试
- **函数**: `readCpuInfo()`, `readMemInfo()`, `readNetDev()` 等

#### 2. **SysInfoPublisherNode** - 发布器节点
- **职责**: ROS 2 节点管理和消息发布
- **特点**: 定时触发，JSON 构建，参数化配置

#### 3. **main** - 程序入口
- **职责**: 初始化 ROS 和启动节点

### 设计优势

- ✅ **职责分离**: 系统读取和 ROS 发布分离
- ✅ **可测试性**: SystemReader 可以独立单元测试
- ✅ **可扩展性**: 易于添加新的系统指标
- ✅ **可配置性**: 通过 ROS 参数灵活配置

---

## 🐛 故障排除

### 问题 1: 权限不足

**现象**: 无法读取某些系统文件

**解决**: 确保程序有权限访问 `/proc` 文件系统（通常不需要 root）

### 问题 2: 发布频率过高导致 CPU 占用

**原因**: `publish_rate_hz` 设置过高

**解决**:
```bash
# 降低发布频率
ros2 run ros2_learning_sysinfo_publisher sysinfo_publisher \
  --ros-args -p publish_rate_hz:=0.5  # 每 2 秒一次
```

---

## 🔗 配套工具

### 可视化查看器

使用 Qt 可视化界面查看系统信息：

```bash
ros2 run ros2_learning_sysinfo_qt_viewer sysinfo_qt_viewer
```

---

## 📚 扩展阅读

- [Linux /proc 文件系统文档](https://www.kernel.org/doc/Documentation/filesystems/proc.txt)
- [ROS 2 参数指南](https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Parameters.html)
- [std_msgs 消息类型](https://docs.ros2.org/latest/api/std_msgs/)

---

## 📝 许可证

Apache 2.0

---

## 👤 维护者

如有问题或建议，请提交 Issue 或 Pull Request。
