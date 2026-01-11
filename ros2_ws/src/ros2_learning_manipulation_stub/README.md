# ros2_learning_manipulation_stub

抓取/放置占位节点，用于任务编排学习与联调。节点提供两个 Trigger 服务，
模拟抓取/放置耗时，返回固定成功结果。

## 提供的服务

- `/manipulation/pick`：模拟抓取
- `/manipulation/place`：模拟放置

## 关键参数

- `operation_time_sec`：模拟动作耗时（秒），默认 `1.0`
- `use_sim_time`：是否使用仿真时间（由 launch 传入）

## 运行方式

构建并 source 后，可直接运行：

```bash
ros2 run ros2_learning_manipulation_stub manipulation_stub
```

也可以使用 launch（会传入 `use_sim_time`）：

```bash
ros2 launch ros2_learning_manipulation_stub manipulation_stub.launch.py use_sim_time:=true
```

## 测试服务（示例）

```bash
ros2 service call /manipulation/pick std_srvs/srv/Trigger
ros2 service call /manipulation/place std_srvs/srv/Trigger
```
