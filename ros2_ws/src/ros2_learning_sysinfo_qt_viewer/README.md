# ROS2 SysInfo Qt Viewer

一个基于 Qt 的 ROS 2 系统信息实时监控界面，展示如何优雅地集成 Qt GUI 框架与 ROS 2 节点。

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Qt](https://img.shields.io/badge/Qt-5%2F6-green)
![C++17](https://img.shields.io/badge/C++-17-orange)

---

## 📋 功能特性

- ✅ **实时系统监控**: 订阅系统信息话题，实时显示 CPU、内存、网络状态
- ✅ **JSON 自动解析**: 智能解析 JSON 数据并格式化为易读的仪表盘视图
- ✅ **Qt 现代界面**: 使用等宽字体和表格布局，清晰展示系统指标
- ✅ **事件循环集成**: 单线程设计，无需信号槽的跨线程通信
- ✅ **容错处理**: JSON 解析失败时友好显示错误信息

---

## 🏗️ 架构设计

### 核心创新：单线程事件循环融合

```
Qt 主事件循环 (QApplication::exec)
  ├─ 处理 UI 事件（鼠标、键盘、绘制）
  └─ 每 10ms 触发 QTimer
       └─ 调用 executor.spin_some()
            └─ 处理 ROS 2 订阅回调
                 └─ 直接更新 UI（线程安全）
```

**优势**:
- 🔹 **简洁**: 避免多线程和信号槽的复杂性
- 🔹 **安全**: 所有 Qt 控件访问在主线程，无竞态条件
- 🔹 **高效**: 低延迟（<20ms）的 UI 更新

### 项目结构

```
ros2_learning_sysinfo_qt_viewer/
├── include/ros2_learning_sysinfo_qt_viewer/
│   ├── json_formatter.hpp      # JSON 解析和格式化工具
│   └── sysinfo_qt_node.hpp     # ROS 2 节点类（集成 Qt）
├── src/
│   ├── json_formatter.cpp      # 格式化实现
│   ├── sysinfo_qt_node.cpp     # 节点实现
│   └── main.cpp                # 主入口（事件循环集成）
├── launch/
│   └── sysinfo_monitor.launch.py  # 一键启动 launch 文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🔧 依赖项

### ROS 2 依赖
- `rclcpp` - ROS 2 C++ 客户端库
- `std_msgs` - 标准消息类型

### 系统依赖
- **Qt 5 或 Qt 6** - GUI 框架
  ```bash
  # Ubuntu/Debian
  sudo apt install qt5-default  # Qt 5
  # 或
  sudo apt install qt6-base-dev  # Qt 6
  ```

---

## 🚀 构建和安装

```bash
# 1. 进入工作空间
cd ~/ros2_ws

# 2. 编译包
colcon build --packages-select ros2_learning_sysinfo_qt_viewer

# 3. source 环境
source install/setup.bash
```

---

## 📖 使用方法

### 🚀 一键启动（推荐）

使用 launch 文件同时启动发布器和查看器：

```bash
ros2 launch ros2_learning_sysinfo_qt_viewer sysinfo_monitor.launch.py
```

这将自动启动：
- ✅ 系统信息发布器（1Hz）
- ✅ Qt 监控界面

按 `Ctrl+C` 关闭所有节点。

---

### 手动分步启动

**步骤 1**: 启动系统信息发布器

```bash
ros2 run ros2_learning_sysinfo_publisher sysinfo_publisher
```

**步骤 2**: 启动 Qt 查看器

```bash
ros2 run ros2_learning_sysinfo_qt_viewer sysinfo_qt_viewer
```

### 自定义话题

默认订阅话题为 `/ros2_learning/sysinfo`，可以通过参数修改：

```bash
ros2 run ros2_learning_sysinfo_qt_viewer sysinfo_qt_viewer \
  --ros-args -p topic:=/custom/sysinfo
```

### 预期输出

Qt 窗口将显示格式化的系统信息：

```
[Received @ 2026-01-21T21:00:00]

=== Summary ===
Host      : wen
OS        : Ubuntu 22.04
Kernel    : 5.15.0-91-generic
Uptime(s) : 123456.78

CPU
  Model : Intel Core i7-9750H
  Cores : 12
  Load  : 1.23 2.34 3.45

Memory (kB)
  Total     : 16384000
  Available : 8192000

Network (bytes)
  IFACE                 RX_BYTES           TX_BYTES
  --------------------------------------------------
  eth0                   123456789          987654321

=== JSON (Indented) ===
{
  "hostname": "wen",
  ...
}
```

---

## 🎨 技术细节

### 1. Qt 与 ROS 2 事件循环集成

**传统方法的问题**:
```cpp
// ❌ 阻塞，Qt 无法启动
rclcpp::spin(node);
app.exec();
```

**我们的解决方案**:
```cpp
// ✅ 融合事件循环
QTimer ros_spin_timer;
QObject::connect(&ros_spin_timer, &QTimer::timeout,
                 [&exec]() { exec.spin_some(); });
ros_spin_timer.start(10);  // 每 10ms

app.exec();  // Qt 主循环
```

### 2. SingleThreadedExecutor 的作用

- **管理回调队列**: 存储所有 ROS 消息和定时器事件
- **非阻塞处理**: `spin_some()` 处理当前消息后立即返回
- **线程安全**: 保证所有回调在 Qt 主线程执行

### 3. JSON 解析和格式化

使用 Qt 的 `QJsonDocument` 进行解析：

```cpp
QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8());
QJsonObject root = doc.object();

// 提取字段
QString hostname = root.value("hostname").toString();
QJsonObject cpu = root.value("cpu").toObject();
```

生成表格式输出：
```cpp
summary += QString("  %1 %2 %3\n")
               .arg(iface.leftJustified(20, ' '))
               .arg(rx.rightJustified(15, ' '))
               .arg(tx.rightJustified(15, ' '));
```

---

## 🐛 故障排除

### 问题 1: 窗口启动但没有数据

**原因**: 没有发布器运行或话题名称不匹配

**解决**:
```bash
# 检查话题
ros2 topic list | grep sysinfo

# 查看话题消息
ros2 topic echo /ros2_learning/sysinfo
```

### 问题 2: 编译错误 "Qt not found"

**原因**: 系统未安装 Qt 或 CMake 找不到 Qt

**解决**:
```bash
# 安装 Qt
sudo apt install qt5-default

# 或指定 Qt 路径
export CMAKE_PREFIX_PATH=/path/to/qt5:$CMAKE_PREFIX_PATH
```

### 问题 3: 界面卡顿

**原因**: ROS 回调处理时间过长

**解决**: 检查 `json_formatter.cpp` 中的处理逻辑，确保高效

---

## 🎓 学习价值

这个项目展示了：

1. ✅ **ROS 2 与 Qt 集成的最佳实践**
2. ✅ **事件循环融合技术**（避免多线程复杂性）
3. ✅ **Executor 的正确使用**（SingleThreadedExecutor）
4. ✅ **专业的 C++ 项目结构**（头文件/实现分离）
5. ✅ **JSON 数据处理**（解析、验证、格式化）

---

## 📚 扩展阅读

- [ROS 2 Executors 文档](https://docs.ros.org/en/rolling/Concepts/About-Executors.html)
- [Qt 事件循环机制](https://doc.qt.io/qt-6/eventsandfilters.html)
- [rclcpp API 参考](https://docs.ros2.org/latest/api/rclcpp/)

---

## 📝 许可证

Apache 2.0

---

## 👤 维护者

如有问题或建议，请提交 Issue 或 Pull Request。
