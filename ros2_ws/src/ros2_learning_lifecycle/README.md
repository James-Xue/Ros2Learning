# ros2_learning_lifecycle

ROS 2 生命周期节点（LifecycleNode）进阶演示包。
通过一个完整的「传感器 → 处理器」数据管线，系统化呈现从状态机设计到多节点编排的全部核心知识点。

---

## 目录

1. [功能概述](#1-功能概述)
2. [架构总览](#2-架构总览)
3. [生命周期状态机详解](#3-生命周期状态机详解)
4. [节点技术细节](#4-节点技术细节)
   - 4.1 [SensorNode](#41-sensornodelifecyclenode)
   - 4.2 [ProcessorNode](#42-processornodelifecyclenode)
   - 4.3 [LifecycleManagerNode](#43-lifecyclemanagernodenode)
5. [关键技术模式与反模式](#5-关键技术模式与反模式)
6. [并发与线程模型](#6-并发与线程模型)
7. [Launch 文件与配置](#7-launch-文件与配置)
8. [运行命令](#8-运行命令)
9. [输入/输出接口](#9-输入输出接口)
10. [验收测试](#10-验收测试)
11. [已知限制](#11-已知限制)

---

## 1. 功能概述

本包演示 ROS 2 生命周期节点体系中最容易出错的三个层面：

| 演示主题 | 涉及节点 | 核心知识点 |
|---|---|---|
| 正确的资源生命周期管理 | `SensorNode` | timer 应在 `on_activate` 创建、`on_deactivate` 销毁，不应依赖 `is_activated()` guard |
| 错误状态注入与 `on_error` 恢复 | `ProcessorNode` | `on_configure` 返回 `FAILURE` vs `ERROR` 的不同状态流转；`on_error` 如何安全恢复到 Unconfigured |
| 多节点自动编排 | `LifecycleManagerNode` | 通过标准 `ChangeState`/`GetState` 服务驱动多个 LifecycleNode；使用独立 CallbackGroup + Executor 线程规避死锁 |

---

## 2. 架构总览

```
┌─────────────────────────────────────────────────────────────────────────┐
│  ros2 launch lifecycle_managed.launch.py  （或 lifecycle_manual）        │
└────────────────────────┬────────────────────────────────────────────────┘
                         │  同一进程（各自独立可执行）
          ┌──────────────┼──────────────────┐
          │              │                  │
  ┌───────▼──────┐ ┌─────▼──────────┐ ┌────▼──────────────────────┐
  │  sensor_node │ │ processor_node │ │  lifecycle_manager_node   │
  │LifecycleNode │ │ LifecycleNode  │ │  (普通 rclcpp::Node)      │
  └──────┬───────┘ └────────┬───────┘ └────────────┬──────────────┘
         │  /sensor_data    │                       │
         │  Float64         │                       │  /sensor_node/change_state
         └──────────────────►                       │  /sensor_node/get_state
                            │ /processed_data       │  /processor_node/change_state
                            │ Float64(移动均值)      │  /processor_node/get_state
                            └───────────────        │
                                                    │  (service 调用)
                         ◄──────────────────────────┘
```

**数据流**：`SensorNode`（Active）→ `/sensor_data` 话题 → `ProcessorNode`（Active）→ `/processed_data` 话题

**控制流**：`LifecycleManagerNode` 通过 `lifecycle_msgs/srv/ChangeState` 服务，顺序驱动两个 LifecycleNode 的状态转换。

**包结构**：

```
ros2_learning_lifecycle/
├── include/ros2_learning_lifecycle/
│   ├── sensor_node.hpp              # SensorNode 声明
│   ├── processor_node.hpp           # ProcessorNode 声明
│   └── lifecycle_manager_node.hpp   # LifecycleManagerNode 声明
├── src/
│   ├── sensor_node.cpp              # 正确的 timer/publisher 生命周期管理
│   ├── processor_node.cpp           # 错误注入 + on_error 恢复
│   └── lifecycle_manager_node.cpp   # 多节点编排 + 多线程服务调用
├── launch/
│   ├── lifecycle_managed.launch.py  # 自动编排模式
│   └── lifecycle_manual.launch.py   # 手动 CLI 控制模式
├── config/
│   └── lifecycle_demo.yaml          # 节点参数默认值
├── test/
│   └── test_lifecycle_transitions.cpp  # 8 个 gtest 测试用例
├── CMakeLists.txt
└── package.xml
```

所有节点同时以 **rclcpp_components 插件**形式注册（`RCLCPP_COMPONENTS_REGISTER_NODE`），既可作为独立可执行文件运行，也可被加载进 `ComponentContainer` 实现进程内零拷贝通信。

---

## 3. 生命周期状态机详解

ROS 2 生命周期节点遵循 [ROS 2 managed nodes 规范](https://design.ros2.org/articles/node_lifecycle.html)，拥有 4 个**主要状态**（Primary States）和 6 个**过渡状态**（Transition States）：

```
                         ┌──────────────────────────────┐
                         │       状态转换图              │
                         └──────────────────────────────┘

                 configure          activate
  Unconfigured ──────────► Inactive ─────────► Active
       ▲                      │   ◄────────────   │
       │         cleanup      │    deactivate     │
       │◄─────────────────────┘                   │
       │                                          │
       │         on_error → SUCCESS               │
       │◄──── ErrorProcessing ◄────── (ERROR)     │
       │                                          │
       │                                          │
       └──── Finalized ◄─── shutdown (from any) ──┘
```

### 关键状态转换规则

| 转换名称 | 触发条件 | 回调函数 | 成功目标状态 | 失败目标状态 |
|---|---|---|---|---|
| `configure` | Unconfigured → | `on_configure()` | Inactive | Unconfigured (FAILURE) / ErrorProcessing (ERROR) |
| `activate` | Inactive → | `on_activate()` | Active | Inactive (FAILURE) / ErrorProcessing (ERROR) |
| `deactivate` | Active → | `on_deactivate()` | Inactive | — |
| `cleanup` | Inactive → | `on_cleanup()` | Unconfigured | — |
| `shutdown` | 任意 → | `on_shutdown()` | Finalized | — |
| *(自动)* | ErrorProcessing → | `on_error()` | Unconfigured (SUCCESS) / Finalized (FAILURE/ERROR) |

### `on_configure` 的三种返回值行为对比

```
on_configure() 返回 SUCCESS → Inactive（正常路径）
on_configure() 返回 FAILURE → Unconfigured（软失败，可重新 configure）
on_configure() 返回 ERROR   → ErrorProcessing → on_error() 触发
                                 └─ on_error 返回 SUCCESS → Unconfigured（可恢复）
                                 └─ on_error 返回 FAILURE → Finalized（不可恢复）
```

`ProcessorNode` 通过参数注入完整演示了以上三条路径。

---

## 4. 节点技术细节

### 4.1 SensorNode（LifecycleNode）

**职责**：模拟传感器，生成正弦波数据并发布到 `/sensor_data`。

**关键设计：资源与状态的绑定关系**

```
状态           资源             说明
─────────────────────────────────────────────────────────
Unconfigured   —（仅参数）      构造函数只声明参数，不创建任何 ROS 实体
Inactive       publisher       on_configure 创建（未激活，不可发布）
Active         publisher+timer on_activate 激活 publisher，再创建 timer
Inactive       publisher       on_deactivate 销毁 timer，停用 publisher
Unconfigured   —               on_cleanup 销毁 publisher
Finalized      —               on_shutdown 销毁所有残余资源
```

**反模式警示**：以下写法是典型错误（anti-pattern）：

```cpp
// ❌ 错误：在 on_configure 创建 timer，然后用 is_activated() 做 guard
void SensorNode::on_configure(...) {
    timer_ = create_wall_timer(period, [this]() {
        if (!is_activated()) return;  // ← 不可靠，且资源提前分配
        publisher_->publish(msg);
    });
}

// ✅ 正确：timer 在 on_activate 创建，on_deactivate 销毁
void SensorNode::on_activate(...) {
    LifecycleNode::on_activate(state);  // 先调用父类
    timer_ = create_wall_timer(period, std::bind(&SensorNode::on_timer, this));
}
void SensorNode::on_deactivate(...) {
    timer_.reset();                      // 先销毁 timer
    LifecycleNode::on_deactivate(state); // 再调用父类
}
```

正确做法保证了：只要 `on_timer` 被调用，节点必然处于 Active 状态；无需在回调中做任何状态检查。

**参数**：

| 参数名 | 类型 | 默认值 | 约束 | 说明 |
|---|---|---|---|---|
| `sensor_id` | string | `"sensor_0"` | — | 日志标识符 |
| `publish_rate_hz` | int | `2` | [1, 10] | 发布频率（Hz） |

---

### 4.2 ProcessorNode（LifecycleNode）

**职责**：订阅 `/sensor_data`，计算滑动窗口均值，发布到 `/processed_data`。

**核心演示：错误状态注入与 `on_error` 恢复**

通过参数 `configure_behavior` 控制 `on_configure` 的返回值，可以在不修改代码的情况下测试三条路径：

```bash
# 正常路径（默认）
ros2 param set /processor_node configure_behavior success

# 软失败路径：FAILURE → 回到 Unconfigured
ros2 param set /processor_node configure_behavior failure

# 错误路径：ERROR → ErrorProcessing → on_error → Unconfigured
ros2 param set /processor_node configure_behavior error
```

**`on_error` 实现分析**：

```cpp
// on_error 的三个职责：
// 1. 清理所有可能被部分初始化的资源（subscription/publisher/window）
// 2. 记录错误日志，便于调试
// 3. 返回 SUCCESS 表示"可恢复"，框架将节点带回 Unconfigured
CallbackReturn ProcessorNode::on_error(const rclcpp_lifecycle::State & state)
{
    cleanup_resources();    // 清理资源
    return CallbackReturn::SUCCESS;  // → Unconfigured
}
```

**订阅回调中的状态守卫**：

订阅（subscription）在 `on_configure` 创建，此后的 Inactive 状态中订阅依然存活、消息依然被接收，但 `publisher_` 处于停用状态。回调中通过 `publisher_->is_activated()` 检查，仅在 Active 状态时发布处理结果，避免向下游投递未授权数据：

```cpp
void ProcessorNode::on_sensor_data(const std_msgs::msg::Float64 & msg) {
    if (!publisher_ || !publisher_->is_activated()) {
        return;  // Inactive 状态：静默丢弃，不发布
    }
    // ... 移动均值计算 ...
    publisher_->publish(out);
}
```

**移动均值算法**：使用 `std::deque<double>` 维护大小为 `window_size` 的滑动窗口，以 `std::accumulate` 计算均值，时间复杂度 O(window_size)。

**参数**：

| 参数名 | 类型 | 默认值 | 约束 | 说明 |
|---|---|---|---|---|
| `configure_behavior` | string | `"success"` | success/failure/error | 错误注入控制 |
| `window_size` | int | `5` | [1, 20] | 移动均值窗口大小 |

---

### 4.3 LifecycleManagerNode（普通 Node）

**职责**：作为普通 `rclcpp::Node`，通过 ROS 2 标准服务接口编排多个 LifecycleNode 的状态转换。

**为什么 LifecycleManagerNode 本身是普通 Node 而非 LifecycleNode？**

管理器的职责是发出控制指令，而非被控制。若管理器本身也是 LifecycleNode，会引入循环依赖（谁来管理管理器？）。生产实践中（如 Nav2 的 `lifecycle_manager`）也遵循此设计。

**启动序列**（`startup_sequence`）：

```
构造函数完成
    │
    └─► 等待 1 秒（给被管理节点启动时间）
            │
            ▼
    等待 change_state / get_state 服务就绪（超时 5s）
            │
            ▼
    顺序 configure 所有节点（检查当前状态，跳过非 Unconfigured 节点）
            │
            ▼
    等待 transition_delay_sec 秒
            │
            ▼
    顺序 activate 所有节点（检查当前状态，跳过非 Inactive 节点）
            │
            ▼
    进入待机（管理器任务完成）
```

**服务编排接口**：

每个 LifecycleNode 自动暴露以下 ROS 2 服务：

```
/<node_name>/get_state        (lifecycle_msgs/srv/GetState)
/<node_name>/change_state     (lifecycle_msgs/srv/ChangeState)
/<node_name>/get_available_states
/<node_name>/get_available_transitions
```

`LifecycleManagerNode` 仅使用前两个，其余供调试工具（`ros2 lifecycle`）使用。

---

## 5. 关键技术模式与反模式

### 5.1 `LifecyclePublisher` 的激活语义

`rclcpp_lifecycle::LifecyclePublisher` 与普通 Publisher 的根本区别在于：未激活状态下调用 `publish()` 是**静默无操作**（no-op），而非发布。
但应避免依赖此行为作为节流手段——正确做法是在 Active 状态才让 timer/callback 能够触发发布。

### 5.2 `on_activate` / `on_deactivate` 中父类调用顺序

```cpp
// on_activate：必须先调用父类（激活所有 LifecyclePublisher），再创建 timer
CallbackReturn SensorNode::on_activate(const rclcpp_lifecycle::State & state) {
    LifecycleNode::on_activate(state);  // ← 必须首先调用
    timer_ = create_wall_timer(...);
    return CallbackReturn::SUCCESS;
}

// on_deactivate：必须先销毁 timer（停止产生发布需求），再调用父类停用 publisher
CallbackReturn SensorNode::on_deactivate(const rclcpp_lifecycle::State & state) {
    timer_.reset();                      // ← 必须首先执行
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
}
```

顺序颠倒会导致：`on_deactivate` 期间 timer 仍在触发，而 publisher 已被停用，发布调用变为 no-op 但 timer 回调依然消耗 CPU。

### 5.3 幂等的 `on_shutdown`

`on_shutdown` 可从**任意状态**触发（Unconfigured、Inactive、Active），因此必须防御性地处理 `nullptr`：

```cpp
CallbackReturn SensorNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
    timer_.reset();     // shared_ptr.reset() 对 nullptr 安全
    publisher_.reset();
    return CallbackReturn::SUCCESS;
}
```

### 5.4 `cleanup_resources()` 的集中化清理

`ProcessorNode` 将资源清理逻辑统一封装在私有函数 `cleanup_resources()` 中，被 `on_cleanup`、`on_shutdown`、`on_error` 三个回调共同调用，遵循 DRY 原则，避免不同路径遗漏清理。

---

## 6. 并发与线程模型

`LifecycleManagerNode` 的线程模型是本包最复杂的部分，也是最容易踩坑的地方。

### 问题根源

`startup_sequence` 需要**阻塞式**等待服务响应（configure、activate 必须顺序执行），而服务响应回调需要 executor 来 spin。如果在已被 executor 管理的节点上再次调用 `rclcpp::spin_until_future_complete(node)`，会触发：

```
std::runtime_error: "Node has already been added to an executor"
```

### 解决方案：独立 CallbackGroup + 独立 Executor 线程

```
主 Executor（ros2 launch 创建）
    ├─ spin sensor_node
    ├─ spin processor_node
    └─ spin lifecycle_manager_node
           └─ [主 CallbackGroup] ← 普通回调

独立 service_executor_（LifecycleManagerNode 创建）
    └─ spin service_cb_group_    ← 仅处理服务响应
           └─ ChangeState 响应
           └─ GetState 响应

startup_thread_（std::thread）
    └─ 调用 change_state() / get_state()
           └─ async_send_request() → 返回 future
           └─ future.wait_for(5s)   ← 阻塞等待
              （service_executor_thread_ 在后台处理响应）
```

关键代码（构造函数中）：

```cpp
// 1. 创建专用 CallbackGroup
service_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

// 2. 服务 Client 绑定到该 CallbackGroup
change_state_client = this->create_client<ChangeState>(
    "/sensor_node/change_state",
    rclcpp::ServicesQoS(),
    service_cb_group_);   // ← 关键

// 3. 将 CallbackGroup 加入独立 Executor（而非整个节点）
service_executor_.add_callback_group(
    service_cb_group_, this->get_node_base_interface());

// 4. 独立线程 spin 该 Executor
service_executor_thread_ = std::thread([this]() {
    service_executor_.spin();
});

// 5. 启动序列在另一个独立线程中等待
startup_thread_ = std::thread([this]() {
    std::this_thread::sleep_for(1s);
    startup_sequence();
});
```

### 析构顺序

析构函数必须保证线程安全的关闭顺序：

```cpp
~LifecycleManagerNode() {
    startup_thread_.join();          // 1. 等待启动序列完成
    service_executor_.cancel();      // 2. 通知 executor 停止
    service_executor_thread_.join(); // 3. 等待 executor 线程退出
}
```

顺序颠倒（先 cancel executor 再等 startup_thread）会导致：startup_thread 中 `future.wait_for()` 永不就绪，死锁。

---

## 7. Launch 文件与配置

### lifecycle_managed.launch.py（推荐入口）

启动全部三个节点，由 `LifecycleManagerNode` 自动完成 configure + activate 序列。

```python
# 节点类型说明：
LifecycleNode(...)  # ← sensor_node / processor_node 使用 LifecycleNode action
Node(...)           # ← lifecycle_manager_node 使用普通 Node action
```

> **注意**：`launch_ros.actions.LifecycleNode` 与 `launch_ros.actions.Node` 的区别在于前者不会自动触发生命周期转换——状态驱动完全由 LifecycleManagerNode 负责。

### lifecycle_manual.launch.py（调试/学习模式）

只启动 `sensor_node` 和 `processor_node`，支持通过 launch 参数覆盖节点参数：

```bash
ros2 launch ros2_learning_lifecycle lifecycle_manual.launch.py \
    sensor_id:=imu_rear \
    publish_rate_hz:=5 \
    configure_behavior:=error   # 触发错误路径测试
```

### config/lifecycle_demo.yaml

```yaml
sensor_node:
  ros__parameters:
    sensor_id: "lidar_front"
    publish_rate_hz: 3          # Hz，范围 [1, 10]

processor_node:
  ros__parameters:
    configure_behavior: "success"  # 改为 "failure" 或 "error" 可测试错误路径
    window_size: 5                 # 移动平均窗口大小，范围 [1, 20]

lifecycle_manager_node:
  ros__parameters:
    managed_nodes: ["sensor_node", "processor_node"]
    transition_delay_sec: 1.0     # configure → activate 之间的等待时间（秒）
```

---

## 8. 运行命令

### 前提条件

```bash
# Source ROS 2 + workspace（从项目根目录执行）
source ./ros2_ws/scripts/source.sh jazzy
```

### 模式一：自动编排（推荐）

```bash
ros2 launch ros2_learning_lifecycle lifecycle_managed.launch.py
```

约 2 秒后节点自动进入 Active 状态，终端输出：

```
[lifecycle_manager_node] ── CONFIGURE ALL ──
[lifecycle_manager_node] 'sensor_node' configure ✓
[lifecycle_manager_node] 'processor_node' configure ✓
[lifecycle_manager_node] 等待 1.0 秒后 activate...
[lifecycle_manager_node] ── ACTIVATE ALL ──
[lifecycle_manager_node] 'sensor_node' activate ✓
[lifecycle_manager_node] 'processor_node' activate ✓
[lifecycle_manager_node] ── 所有节点已 Active ── 管理器进入待机状态
```

**观察数据流**（另开终端）：

```bash
# 观察原始传感器数据（正弦波）
ros2 topic echo /sensor_data

# 观察处理后数据（移动均值，应比原始数据平滑）
ros2 topic echo /processed_data

# 确认节点状态
ros2 lifecycle list /sensor_node
ros2 lifecycle list /processor_node
```

### 模式二：手动 CLI 控制

```bash
# 终端 A：启动节点（初始为 Unconfigured）
ros2 launch ros2_learning_lifecycle lifecycle_manual.launch.py

# 终端 B：手动驱动状态转换
ros2 lifecycle set /sensor_node configure
ros2 lifecycle set /processor_node configure

ros2 lifecycle set /sensor_node activate
ros2 lifecycle set /processor_node activate

# 暂停数据流（节点回到 Inactive）
ros2 lifecycle set /sensor_node deactivate
ros2 lifecycle set /processor_node deactivate

# 重置（回到 Unconfigured）
ros2 lifecycle set /sensor_node cleanup
ros2 lifecycle set /processor_node cleanup

# 彻底关闭
ros2 lifecycle set /sensor_node shutdown
ros2 lifecycle set /processor_node shutdown
```

### 模式三：错误路径测试

```bash
# 注入 FAILURE（configure 失败，回到 Unconfigured）
ros2 launch ros2_learning_lifecycle lifecycle_manual.launch.py configure_behavior:=failure
ros2 lifecycle set /processor_node configure
# 预期：节点仍在 Unconfigured（configure 未成功）

# 注入 ERROR（触发 on_error 恢复）
ros2 launch ros2_learning_lifecycle lifecycle_manual.launch.py configure_behavior:=error
ros2 lifecycle set /processor_node configure
# 预期日志：
#   [ProcessorNode] on_configure → ERROR（注入）
#   [ProcessorNode] on_error 触发！...正在清理并尝试恢复...
#   [ProcessorNode] on_error 恢复成功 → 节点将回到 Unconfigured
ros2 lifecycle list /processor_node
# 预期：当前状态为 unconfigured（可重新 configure）
```

---

## 9. 输入/输出接口

### 话题

| 话题名 | 方向 | 消息类型 | 发布节点 | 说明 |
|---|---|---|---|---|
| `/sensor_data` | 发布 | `std_msgs/Float64` | SensorNode（Active 时） | 正弦波模拟值，频率由 `publish_rate_hz` 控制 |
| `/processed_data` | 发布 | `std_msgs/Float64` | ProcessorNode（Active 时） | `window_size` 点滑动均值 |

### 服务（LifecycleNode 自动暴露）

| 服务名 | 消息类型 | 说明 |
|---|---|---|
| `/<node>/change_state` | `lifecycle_msgs/srv/ChangeState` | 触发状态转换 |
| `/<node>/get_state` | `lifecycle_msgs/srv/GetState` | 查询当前状态 |
| `/<node>/get_available_states` | `lifecycle_msgs/srv/GetAvailableStates` | 查询所有可能状态 |
| `/<node>/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions` | 查询当前可用转换 |

### 参数

**sensor_node**：

| 参数 | 类型 | 默认值 | 约束 |
|---|---|---|---|
| `sensor_id` | string | `"lidar_front"` | — |
| `publish_rate_hz` | int | `3` | [1, 10] |

**processor_node**：

| 参数 | 类型 | 默认值 | 约束 |
|---|---|---|---|
| `configure_behavior` | string | `"success"` | `success`/`failure`/`error` |
| `window_size` | int | `5` | [1, 20] |

**lifecycle_manager_node**：

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `managed_nodes` | string[] | `["sensor_node","processor_node"]` | 被管理节点名列表 |
| `transition_delay_sec` | double | `1.0` | configure → activate 间隔（秒） |

---

## 10. 验收测试

### 构建与运行测试

```bash
# 从工作空间根目录
cd /root/Ros2Learning/ros2_ws

# 构建（含测试目标）
colcon build --packages-select ros2_learning_lifecycle --symlink-install

# 运行测试
colcon test --packages-select ros2_learning_lifecycle
colcon test-result --verbose
```

### 测试用例说明（8 个，全部通过）

| # | 测试名 | 验证内容 |
|---|---|---|
| 1 | `SensorNodeConfigureSuccess` | configure 后状态变为 Inactive |
| 2 | `SensorNodeFullCycle` | 完整正向循环：configure → activate → deactivate → cleanup → Unconfigured |
| 3 | `SensorNodeShutdownFromActive` | 从 Active 状态 shutdown → Finalized |
| 4 | `SensorPublisherInactiveAfterConfigure` | configure 后 publisher 存在但未激活 |
| 5 | `SensorPublisherActiveAfterActivate` | activate 后 publisher 处于激活状态 |
| 6 | `SensorPublisherInactiveAfterDeactivate` | deactivate 后 publisher 停用 |
| 7 | `ProcessorNodeFailureOnConfigure` | `configure_behavior=failure` → configure 失败 → 状态为 Unconfigured |
| 8 | `ProcessorNodeErrorOnConfigure` | `configure_behavior=error` → on_error 触发 → 状态恢复为 Unconfigured |

测试直接调用 `trigger_transition()` API，不依赖 ROS 2 launch 或网络，执行速度快且确定性高。

---

## 11. 已知限制

1. **`LifecycleManagerNode` 无重试机制**：若某节点 configure/activate 失败，管理器直接放弃后续步骤，不会尝试重试或部分回滚。生产场景需补充指数退避重试逻辑。

2. **无双向依赖编排**：`ProcessorNode` 依赖 `SensorNode` 的数据，但 Manager 对此一无所知——即使 `SensorNode` activate 失败，Manager 仍会尝试 activate `ProcessorNode`。生产实现（如 Nav2）会在每次 activate 后验证数据流健康度。

3. **仅演示正向启动**：Manager 目前只实现了启动序列（configure + activate），未实现关闭序列（deactivate + cleanup + shutdown）。

4. **`publish_rate_hz` 变更需重启**：该参数在 `on_activate` 读取后固定，运行中修改不生效，需 deactivate → activate 循环。

5. **模拟数据无单位语义**：`/sensor_data` 发布的是纯粹的正弦波数值，仅用于演示，不代表任何真实物理量。
