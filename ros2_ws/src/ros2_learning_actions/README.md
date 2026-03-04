# ros2_learning_actions

ROS 2 Action 通信机制进阶演示，基于 `CountTask.action` 实现完整的 Action Server 与 Client。

## 目录

1. [功能概述](#1-功能概述)
2. [架构总览](#2-架构总览)
3. [Action 通信模型详解](#3-action-通信模型详解)
4. [节点技术细节](#4-节点技术细节)
5. [关键技术模式](#5-关键技术模式)
6. [Launch 文件与配置](#6-launch-文件与配置)
7. [运行命令](#7-运行命令)
8. [输入/输出接口](#8-输入输出接口)
9. [验收测试](#9-验收测试)
10. [已知限制](#10-已知限制)

---

## 1. 功能概述

| 演示主题 | 涉及节点 | 核心知识点 |
|----------|----------|------------|
| Action Server 完整实现 | `CountActionServer` | `rclcpp_action::create_server`、Goal 校验（ACCEPT/REJECT）、`std::thread` 异步执行、Feedback 发布、取消处理 |
| Action Client 完整实现 | `CountActionClient` | `rclcpp_action::create_client`、三回调（goal_response/feedback/result）、参数化目标、超时取消演示 |
| 接口定义 | — | `CountTask.action`（Goal/Feedback/Result 三段式） |

---

## 2. 架构总览

### 通信架构

```
┌─────────────────────┐         /count_task          ┌─────────────────────┐
│  CountActionClient  │ ──── Goal(target,period) ──→ │  CountActionServer  │
│                     │ ←── Feedback(current,%) ──── │                     │
│  参数：             │ ←── Result(count,time,ok) ── │  校验：             │
│   target            │                              │   target > 0        │
│   period_sec        │         async_cancel         │   period_sec > 0    │
│   cancel_after_sec  │ ──── cancel_goal ─────────→  │                     │
└─────────────────────┘                              └─────────────────────┘
```

### 包文件结构

```
ros2_learning_actions/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── count_action_demo.yaml       # 默认参数
├── include/ros2_learning_actions/
│   ├── count_action_server.hpp       # Server 节点声明
│   └── count_action_client.hpp       # Client 节点声明
├── src/
│   ├── count_action_server.cpp       # Server 实现
│   └── count_action_client.cpp       # Client 实现
├── launch/
│   └── count_action_demo.launch.py   # Server + Client 启动文件
└── test/
    └── test_count_action.cpp         # 8 个 GTest 用例
```

---

## 3. Action 通信模型详解

### CountTask.action 三段式结构

```
# Goal（客户端 → 服务端）
int32   target       # 计数上限
float32 period_sec   # 每次递增间隔（秒）
---
# Result（服务端 → 客户端，任务结束时）
int32   final_count  # 实际达到的计数值
float32 elapsed_sec  # 总耗时
bool    succeeded    # true=正常完成，false=取消或出错
---
# Feedback（服务端 → 客户端，执行过程中）
int32   current      # 当前计数值
float32 percent      # 完成百分比 [0.0, 100.0]
```

### Action 协议时序

```
Client                           Server
  │                                │
  │──── SendGoal(target,period) ──→│
  │                                │── handle_goal(): 校验参数
  │←── GoalResponse(ACCEPT) ──────│
  │                                │── handle_accepted(): 启动 std::thread
  │                                │
  │←── Feedback(1, 20%) ─────────│── execute(): sleep → publish_feedback
  │←── Feedback(2, 40%) ─────────│
  │←── Feedback(3, 60%) ─────────│
  │                                │
  │   [可选] CancelGoal ─────────→│── handle_cancel(): ACCEPT
  │                                │── execute(): is_canceling() → canceled()
  │                                │
  │←── Result(count,time,ok) ────│── succeed() 或 canceled()
  │                                │
```

### GoalResponse 枚举

| 值 | 含义 |
|----|------|
| `REJECT` | 参数不合法，拒绝执行 |
| `ACCEPT_AND_EXECUTE` | 参数合法，立即开始执行 |
| `ACCEPT_AND_DEFER` | 接受但延迟执行（本包未使用） |

### ResultCode 枚举

| 值 | 含义 |
|----|------|
| `SUCCEEDED` | 正常完成 |
| `CANCELED` | 被客户端取消 |
| `ABORTED` | 服务端主动中止 |
| `UNKNOWN` | 未知状态 |

---

## 4. 节点技术细节

### 4.1 CountActionServer

**继承**：`rclcpp::Node`（非生命周期节点，降低学习复杂度）

**四个必须实现的回调**：

| 回调 | 触发时机 | 本包行为 |
|------|----------|----------|
| `handle_goal` | 收到 Goal 请求 | 校验 target>0 且 period_sec>0，否则 REJECT |
| `handle_cancel` | 收到取消请求 | 无条件 ACCEPT |
| `handle_accepted` | Goal 被接受后 | join 旧线程 → 启动新 `std::thread(execute)` |
| `execute` | 在独立线程中 | for 循环 1..target，sleep → 检查取消 → 发 feedback |

**执行流程**：

```
handle_accepted()
  └─ std::thread(execute, goal_handle)
       ├─ for current = 1 → target:
       │    ├─ 检查 is_canceling() → cancel_with_result()
       │    ├─ sleep_for(period_sec)
       │    ├─ 再次检查 is_canceling()
       │    └─ publish_feedback(current, percent)
       └─ succeed_with_result(target, elapsed)
```

### 4.2 CountActionClient

**参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `target` | int | 10 | 计数目标 |
| `period_sec` | double | 0.5 | 每次计数间隔 |
| `cancel_after_sec` | double | -1.0 | 正值=超时取消，负值=不取消 |

**三个回调**：

| 回调 | 触发时机 | 本包行为 |
|------|----------|----------|
| `goal_response_callback` | Goal 被接受/拒绝 | 保存 handle，调度取消 timer |
| `feedback_callback` | 收到 Feedback | RCLCPP_INFO 打印进度 |
| `result_callback` | 收到 Result | switch(ResultCode) 打印结果 |

---

## 5. 关键技术模式

### 5.1 execute 必须在独立线程中

`handle_accepted` 是 ROS 回调上下文中的非阻塞函数。如果在其中直接执行耗时的计数循环，会阻塞 executor 线程，导致：
- 无法处理新的 Goal 请求
- 无法处理取消请求
- Feedback 无法被发送（waitable 无法被处理）

正确做法：`std::thread` 独立执行，executor 继续处理回调。

### 5.2 双重取消检查

```cpp
for (int32_t current = 1; current <= target; ++current) {
    if (goal_handle->is_canceling()) { ... }  // 等待前检查
    std::this_thread::sleep_for(period);
    if (goal_handle->is_canceling()) { ... }  // 等待后检查
    publish_feedback(...);
}
```

等待前检查避免了"已经取消但还要等一个 period"的延迟；等待后检查覆盖了"sleep 期间收到取消"的场景。

### 5.3 Client 的 one-shot timer 延迟初始化

```cpp
init_timer_ = create_wall_timer(100ms, [this]() {
    init_timer_->cancel();  // one-shot
    send_goal();
});
```

不能在构造函数中调用 `wait_for_action_server()`，因为 executor 可能尚未 spin。使用 timer 延迟到 executor 启动后再发送。

---

## 6. Launch 文件与配置

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target` | 5 | 计数目标 |
| `period_sec` | 0.3 | 每次计数间隔 |
| `cancel_after_sec` | -1.0 | 取消超时（负值=不取消） |

### YAML 配置（`config/count_action_demo.yaml`）

> 注：launch 文件通过 `LaunchConfiguration` 传递参数，不自动加载此 YAML。
> 此文件用于 `ros2 run` 手动启动时通过 `--params-file` 加载。

```yaml
count_action_server:
  ros__parameters: {}

count_action_client:
  ros__parameters:
    target: 5
    period_sec: 0.3
    cancel_after_sec: -1.0
```

---

## 7. 运行命令

```bash
# 构建
cd /root/Ros2Learning && ./ros2_ws/scripts/build.sh jazzy

# source 工作空间
source ./ros2_ws/scripts/source.sh jazzy

# 默认模式（target=5, period=0.3s, 不取消）
ros2 launch ros2_learning_actions count_action_demo.launch.py

# 自定义参数
ros2 launch ros2_learning_actions count_action_demo.launch.py \
    target:=10 period_sec:=0.2

# 演示取消路径（2 秒后自动取消）
ros2 launch ros2_learning_actions count_action_demo.launch.py \
    target:=20 period_sec:=0.5 cancel_after_sec:=2.0

# 手动 CLI 发送 goal（不使用 Client 节点）
ros2 action send_goal /count_task \
    ros2_learning_custom_interfaces/action/CountTask \
    "{target: 5, period_sec: 0.2}" --feedback

# 查看 action 信息
ros2 action list
ros2 action info /count_task -t
```

### 预期输出（默认模式）

```
[count_action_server] [CountActionServer] 已启动，action: /count_task
[count_action_client] [CountActionClient] 发送 goal: target=5, period_sec=0.300
[count_action_server] [CountActionServer] ACCEPT: target=5, period_sec=0.300
[count_action_client] [CountActionClient] goal 已被接受，等待执行中...
[count_action_client] [CountActionClient] 进度: current=1 (20.0%)
[count_action_client] [CountActionClient] 进度: current=2 (40.0%)
[count_action_client] [CountActionClient] 进度: current=3 (60.0%)
[count_action_client] [CountActionClient] 进度: current=4 (80.0%)
[count_action_client] [CountActionClient] 进度: current=5 (100.0%)
[count_action_server] [CountActionServer] 成功完成：final_count=5, elapsed=1.502s
[count_action_client] [CountActionClient] 成功！final_count=5, elapsed=1.502s
```

---

## 8. 输入/输出接口

### Action 接口

| 名称 | 类型 | 方向 |
|------|------|------|
| `/count_task` | `ros2_learning_custom_interfaces/action/CountTask` | Server ↔ Client |

### CountActionClient 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `target` | int | 10 | 计数目标（Server 校验 > 0） |
| `period_sec` | double | 0.5 | 每次计数间隔（Server 校验 > 0） |
| `cancel_after_sec` | double | -1.0 | 正值=超时取消延迟，负值=不取消 |

---

## 9. 验收测试

### 构建与运行测试

```bash
cd /root/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_actions --symlink-install
colcon test --packages-select ros2_learning_actions
colcon test-result --verbose
```

### 测试用例

| 用例 | 验证点 |
|------|--------|
| `ServerAcceptsValidGoal` | 合法参数（target=2, period=0.05）→ goal 被接受并完成 |
| `ServerRejectsZeroTarget` | target=0 → REJECT |
| `ServerRejectsZeroPeriod` | period_sec=0 → REJECT |
| `ServerRejectsNegativePeriod` | period_sec=-0.5 → REJECT |
| `ServerRejectsNegativeTarget` | target=-1 → REJECT |
| `ServerSucceedsWithCorrectResult` | target=3 → SUCCEEDED, final_count=3, succeeded=true |
| `ServerHandlesCancelRequest` | 发送后立即取消 → CANCELED, succeeded=false |
| `FeedbackFieldsAreCorrect` | feedback.current 单调递增，percent 在 [0,100] |

### 测试架构

- **Fixture**：`MultiThreadedExecutor` 后台 spin + 匿名 action client
- **同步机制**：`shared_ptr<promise>/future`，无 sleep 轮询
- **隔离**：每个用例独享 `rclcpp::init/shutdown` 周期

---

## 10. 已知限制

1. **单目标模式**：不支持并发 Goal，`handle_accepted` 会 join 前一个执行线程后再启动新线程
2. **sleep 精度**：`std::this_thread::sleep_for` 精度依赖操作系统调度，实际 `elapsed_sec` 可能略大于 `target * period_sec`
3. **无重试机制**：Goal 被拒绝后 Client 不会重试，需用户修改参数后重新发送
4. **测试隔离**：每个 TEST_F 都执行 `rclcpp::init/shutdown`，无法在同一进程中复用节点实例
5. **非 Composition 模式**：Launch 文件以独立进程方式启动 Server 和 Client，未使用 `ComposableNodeContainer`
