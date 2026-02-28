# ros2_learning_manipulation_stub

> 抓取/放置占位节点，用于任务编排学习与联调；提供两个 `std_srvs/Trigger` 服务，以阻塞 sleep 模拟操作耗时，始终返回成功。

## 功能描述

本包实现一个最小化的机械臂操作桩节点（`manipulation_stub`），对外暴露 `/manipulation/pick` 和 `/manipulation/place` 两个服务，供 `ros2_learning_task_runner` 等任务编排节点调用。节点接到请求后，阻塞等待 `operation_time_sec` 秒后返回 `success=true`，无需真实的机械臂硬件即可完成完整的任务流程联调。

## 运行命令

```bash
# 直接运行（使用系统时间）
ros2 run ros2_learning_manipulation_stub manipulation_stub

# 使用 launch 文件启动（默认传入 use_sim_time:=true）
ros2 launch ros2_learning_manipulation_stub manipulation_stub.launch.py

# 指定是否使用仿真时间
ros2 launch ros2_learning_manipulation_stub manipulation_stub.launch.py use_sim_time:=false

# 手动调用服务验证
ros2 service call /manipulation/pick std_srvs/srv/Trigger
ros2 service call /manipulation/place std_srvs/srv/Trigger
```

## 输入/输出

### 提供的服务

| 服务名 | 类型 | 说明 |
|---|---|---|
| `/manipulation/pick` | `std_srvs/srv/Trigger` | 模拟抓取；无请求字段，响应 `success=true, message="pick ok"` |
| `/manipulation/place` | `std_srvs/srv/Trigger` | 模拟放置；无请求字段，响应 `success=true, message="place ok"` |

### 关键参数

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `operation_time_sec` | `double` | `1.0` | 模拟动作阻塞耗时（秒） |
| `use_sim_time` | `bool` | `true`（launch 传入） | 是否使用仿真时钟 |

## 验收测试

暂无，待补充。

## 已知限制

- 服务处理函数为阻塞调用（`rclcpp::sleep_for`），在等待期间节点不响应其他回调；如需并发调用，应改为多线程执行器。
- 始终返回成功，无法模拟失败场景；如需测试任务编排的错误恢复逻辑，需手动修改返回值。
