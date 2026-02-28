# ros2_learning_control_demo

> 演示如何为 ros2_control 框架编写自定义 Hardware Interface 插件，并通过 ForwardCommandController 控制一个虚拟单关节机器人。

## 功能描述

本包是学习 ros2_control 框架的最小化示例，聚焦于"硬件接口层"的实现，不依赖任何真实电机或传感器。

**自定义 Hardware Interface**：`SimpleJointHardware` 继承自 `hardware_interface::SystemInterface`，实现了 ros2_control 要求的五个核心接口：`on_init`（解析 URDF 中的 `<ros2_control>` 配置）、`export_state_interfaces`（向控制器暴露 `joint1/position` 状态接口）、`export_command_interfaces`（向控制器暴露 `joint1/position` 命令接口）、`read`（模拟一阶低通滤波器，让关节以时间常数 τ=0.1 s 逐渐收敛到目标位置）、`write`（虚拟硬件无需实际写出，函数体为空）。该插件通过 `pluginlib` 机制注册，由 Controller Manager 在运行时动态加载。

**控制器配置**：本包使用两个标准控制器。`joint_state_broadcaster` 读取 StateInterface 并以 50 Hz 向 `/joint_states` 发布关节状态，同时驱动 TF。`forward_command_controller` 订阅 `/forward_position_controller/commands`，将收到的 `Float64MultiArray` 直接写入 CommandInterface，实现开环位置直传。Controller Manager 以 100 Hz 控制频率运行。

**机器人模型**：URDF 描述了一个极简的两连杆机器人——固定的 `base_link` 与绕 Z 轴旋转的 `joint1`（限位 ±π rad），`<ros2_control>` 标签将 `SimpleJointHardware` 插件绑定到该关节。RViz2 配置文件已预置，启动后可直接看到关节随命令转动的三维可视化。

学习要点：`SystemInterface` 的生命周期回调顺序、`StateInterface`/`CommandInterface` 的导出与共享内存机制、`pluginlib` 插件注册流程（`.xml` 描述文件 + `PLUGINLIB_EXPORT_CLASS` 宏）、`spawner` 工具的用法。

## 运行命令

```bash
# 1. 环境准备（每个终端均需执行）
source /opt/ros/jazzy/setup.bash
source /root/Ros2Learning/ros2_ws/install/setup.bash

# 2. 启动完整演示（Controller Manager + 两个控制器 + RViz2）
ros2 launch ros2_learning_control_demo demo.launch.py

# 3. 在另一个终端发送位置命令（单位：弧度）
# 将 joint1 移动到 1.0 rad
ros2 topic pub /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [1.0]" --once

# 移动到 -1.5 rad
ros2 topic pub /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [-1.5]" --once

# 4. 查看关节状态
ros2 topic echo /joint_states

# 5. 查看已加载的控制器列表
ros2 control list_controllers

# 6. 查看已注册的硬件接口
ros2 control list_hardware_interfaces
```

## 输入/输出

### 话题

| 方向 | 话题名 | 消息类型 | 发布者/订阅者 | 说明 |
|---|---|---|---|---|
| 订阅 | `/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | ForwardCommandController | 接收目标位置命令（弧度），`data[0]` 对应 `joint1` |
| 发布 | `/joint_states` | `sensor_msgs/msg/JointState` | JointStateBroadcaster | 以 50 Hz 发布 `joint1` 的当前位置 |
| 发布 | `/robot_description` | `std_msgs/msg/String` | RobotStatePublisher | 发布 URDF 字符串，供 RViz2 和 TF 使用 |
| 发布 | `/tf` / `/tf_static` | `tf2_msgs/msg/TFMessage` | RobotStatePublisher | 发布 `base_link -> link1` 的变换 |

### 服务

| 服务名 | 类型 | 说明 |
|---|---|---|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | 列出已加载控制器 |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | 切换控制器激活状态 |

### 控制器参数（来自 `config/controllers.yaml`）

| 参数 | 值 | 说明 |
|---|---|---|
| `controller_manager.update_rate` | 100 Hz | 控制循环频率 |
| `joint_state_broadcaster.state_publish_rate` | 50.0 Hz | 关节状态发布频率 |
| `forward_position_controller.joints` | `[joint1]` | 受控关节列表 |
| `forward_position_controller.interface_name` | `position` | 使用位置接口 |

### 硬件接口（`SimpleJointHardware`）

| 接口类型 | 接口名 | 说明 |
|---|---|---|
| StateInterface | `joint1/position` | 暴露给控制器的当前关节位置（rad） |
| CommandInterface | `joint1/position` | 控制器写入的目标关节位置（rad） |

## 验收测试

本包启用了 ament lint 测试套件（在 `BUILD_TESTING` 模式下）：

```bash
cd /root/Ros2Learning/ros2_ws
colcon test --packages-select ros2_learning_control_demo
colcon test-result --verbose
```

当前配置的测试目标：

| 测试目标 | 工具 | 说明 |
|---|---|---|
| `test_uncrustify` | `ament_cmake_uncrustify` | C++ 代码格式检查（依据 `uncrustify.cfg`） |
| `ament_copyright` | `ament_cmake_copyright` | 源文件版权声明检查 |
| `ament_cpplint` | `ament_cmake_cpplint` | C++ 代码风格检查 |
| `ament_flake8` | `ament_cmake_flake8` | Python 代码风格检查 |
| `ament_pep257` | `ament_cmake_pep257` | Python docstring 格式检查 |
| `ament_xmllint` | `ament_cmake_xmllint` | XML 文件格式检查 |
| `ament_lint_cmake` | `ament_cmake_lint_cmake` | CMakeLists.txt 格式检查 |

预期：全部 lint 测试通过。暂无功能性单元测试，待补充。

## 已知限制

- `SimpleJointHardware` 当前仅支持**单个关节**，若 URDF 中配置了多个关节，`on_init` 会直接返回 ERROR 并阻止系统启动。
- `read()` 中使用一阶低通滤波模拟关节运动（时间常数 τ = 0.1 s），这是纯内存计算，与真实电机驱动行为不同，不可用于运动学标定。
- 本包不包含速度或力矩接口，若需要切换到 `velocity_controllers/JointGroupVelocityController`，需要在 URDF 和 `controllers.yaml` 中同步新增对应接口声明。
- 启动时 RViz2 会同步打开，在无显示环境（如 WSL2 无 X Server 转发）下需将 launch 文件中的 `rviz_node` 注释掉，或使用 `DISPLAY` 环境变量配置图形输出。
- 需要预先安装：`ros-jazzy-ros2-control`、`ros-jazzy-ros2-controllers`、`ros-jazzy-robot-state-publisher`、`ros-jazzy-xacro`、`ros-jazzy-rviz2`。
