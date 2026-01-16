# 机械臂仿真快速启动指南

欢迎！这个文档将帮助你从零开始搭建ROS2机械臂仿真环境并快速上手。

---

## 📦 安装依赖包

在开始之前，需要安装必要的ROS2软件包：

```bash
# 更新软件源
sudo apt update

# 安装MoveIt 2核心框架和规划器
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-planners \
  ros-jazzy-moveit-plugins \
  ros-jazzy-rviz-visual-tools

# 安装Panda机械臂模型和配置
sudo apt install -y \
  ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-moveit-resources-panda-description

# 安装必要的控制器（重要！）
sudo apt install -y \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-controller-manager \
  ros-jazzy-gripper-controllers \
  ros-jazzy-position-controllers
```

**注意**：如果你使用的是ROS Humble或其他版本，将上述命令中的`jazzy`替换为你的ROS版本名称。

---

## ✅ 安装验证

安装完成后，你应该已经拥有以下内容：

- **MoveIt 2**: 机械臂运动规划框架
- **Panda机械臂**: Franka Emika工业机械臂仿真模型  
- **ros2_learning_arm_basics**: 你的第一个机械臂控制包

---

## 🚀 快速开始

### 方式一：手动控制机械臂（推荐入门）

这个方式会启动RViz2，你可以用鼠标拖动机械臂。

```bash
# 进入工作空间
cd ~/Ros2Learning/ros2_ws

# 启动Panda仿真（使用我们准备好的脚本）
./start_panda_simulation.sh
```

**操作说明**：
1. RViz2窗口会自动打开
2. 你会看到橙色的Panda机械臂
3. 机械臂末端有一个**蓝色球形标记**（交互式标记）
4. 用鼠标**拖动蓝色球**到新位置
5. 点击 **"Plan & Execute"** 按钮
6. 机械臂会自动规划路径并移动！

---

### 方式二：使用代码控制机械臂

运行我们刚刚编写的控制节点，机械臂会自动执行演示动作。

**步骤1：在第一个终端启动MoveIt**
```bash
cd ~/Ros2Learning/ros2_ws
./start_panda_simulation.sh
```

**步骤2：在第二个终端运行控制节点**
```bash
cd ~/Ros2Learning/ros2_ws
source install/setup.bash
ros2 run ros2_learning_arm_basics arm_position_controller
```

**你会看到**：
- 机械臂先移动到"ready"姿态
- 然后移动到自定义的笛卡尔位置
- 最后返回"ready"姿态

---

## 📝 代码解释

### 核心控制类：ArmPositionController

位置：`src/ros2_learning_arm_basics/src/arm_position_controller.cpp`

**主要功能**：

1. **移动到预定义姿态**
   ```cpp
   moveToNamedTarget("ready");  // 移动到ready姿态
   ```

2. **移动到指定笛卡尔位置**
   ```cpp
   geometry_msgs::msg::Pose target_pose;
   target_pose.position.x = 0.3;
   target_pose.position.y = 0.2;
   target_pose.position.z = 0.5;
   moveToPose(target_pose);
   ```

3. **直接控制关节角度**
   ```cpp
   std::vector<double> joint_values = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
   moveJoints(joint_values);
   ```

---

## 🔧 下一步学习

### 1. 修改演示代码
尝试修改 `arm_position_controller.cpp` 中的 `runDemo()` 函数：
- 改变目标位置的坐标
- 添加更多的运动点
- 调整延迟时间

### 2. 查看机械臂的关节名称
```bash
# 查看关节状态话题
ros2 topic echo /joint_states
```

### 3. 查看TF变换树
```bash
# 生成TF树
ros2 run tf2_tools view_frames

# 查看生成的PDF
evince frames.pdf  # 或者用你喜欢的PDF查看器
```

### 4. 探索MoveIt规划器
RViz2的MotionPlanning面板中有很多选项可以探索：
- **Planning Group**: 选择要控制的机械臂部分
- **Query Goal State**: 设置目标状态
- **Planning Library**: 尝试不同的规划算法（RRT, RRTConnect等）

---

## 🎯 常见问题

### Q: RViz2窗口打不开？
A: 确保你的系统支持图形化界面。如果是远程服务器，需要配置X11转发。

### Q: 机械臂规划失败？
A: 可能的原因：
1. 目标位置超出机械臂工作空间
2. 目标姿态无法到达（逆运动学无解）
3. 路径中有碰撞

尝试调整目标位置到更保守的值。

### Q: "panda_arm"规划组找不到？
A: 确保MoveIt demo已经启动。控制节点需要MoveIt的move_group节点在运行。

---

## 📚 进阶资源

1. **MoveIt 2官方教程**  
   https://moveit.picknik.ai/jazzy/index.html

2. **Panda机械臂URDF**  
   `/opt/ros/jazzy/share/moveit_resources_panda_description/urdf/panda.urdf`

3. **你的学习计划**  
   参考 `arm_simulation_learning_plan.md` 中的详细路线图

---

## 🎉 恭喜！

你已经成功：
- ✅ 搭建了完整的机械臂仿真环境
- ✅ 启动了第一个机械臂仿真
- ✅ 编写并运行了控制代码

继续按照学习计划深入探索机械臂控制吧！有问题随时问我。🚀
