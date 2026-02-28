# ros2_learning_kinematics

> 机器人运动学学习包，实现平面二自由度几何法 IK 和六轴球形手腕 D-H 参数正/逆运动学，附 ROS 2 演示节点与 gtest 单元测试。

# ROS 2 机器人运动学 (Kinematics) 深度解析

本项目是一个面向初学者的机器人运动学学习包，涵盖了从简单的平面机构到工业级六轴机械臂的核心算法实现。

---

## 📂 功能模块

本仓库通过两个核心求解器，演示了机器人学中最重要的数学基石：

### 1. 平面二自由度求解器 (`Planar2DOF`)
- **场景**：模拟工作在二维平面上的连杆机构（如绘图仪、SCARA 机器人的前两轴）。
- **正运动学 (FK)**：通过简单的三角函数计算末端坐标。
- **逆运动学 (IK)**：使用**几何法（余弦定理）**求解。
  - 完美演示了“工作空间检查”（能不能够得着）。
  - 演示了“多解现象”：同一个目标点，机械臂可以有“肘上（Elbow Up）”和“肘下（Elbow Down）”两种姿态。

### 2. 六轴球形手腕求解器 (`SphericalWrist6DOF`)
- **场景**：模拟典型的工业六轴机械臂（如 PUMA 560、UR 协作臂）。
- **核心技术：运动学解耦 (Kinematic Decoupling)**
  - 将复杂的 6D 空间问题拆解为 3D 位置解 + 3D 姿态解。
- **正运动学**：演示了 **D-H 参数矩阵连乘法** 的数学魅力。
- **逆运动学**：演示了如何通过“回退一个工具长度”找到手腕中心点，从而将 6 轴难题降维打击。

---

## 📐 核心知识点：D-H 参数 (Denavit-Hartenberg)

本项目深入浅出地演示了 D-H 参数的 4 个关键动作：
1. **$\theta$ (Theta)**: 绕 Z 轴旋转（关节电机转动）。
2. **$d$ (d)**: 沿 Z 轴平移（对齐高度）。
3. **$a$ (a)**: 沿 X 轴平移（骨头长度）。
4. **$\alpha$ (Alpha)**: 绕 X 轴旋转（扭转关节方向）。

通过这 4 个参数构成的 $4 \times 4$ 齐次变换矩阵，我们可以描述世界上任何串联机械臂的运动。

---

## 🚀 运行演示 (2-DOF Demo)

1. **编译包**：
   ```bash
   colcon build --packages-select ros2_learning_kinematics
   source install/setup.bash
   ```

2. **运行测试节点**：
   ```bash
   ros2 run ros2_learning_kinematics ik_demo_node
   ```

**实验现象**：
节点会每 2 秒随机生成一个目标点，调用逆运动学算回关节角，并再次通过正运动学验证误差。你可以在终端看到极高精度的解算结果。

---

## 输入/输出

### ik_demo_node

该节点为纯演示节点，不订阅任何外部话题，也不发布话题或提供服务。通过内部定时器（2 秒周期）随机生成目标点并在日志中输出计算结果。

| 接口类型 | 名称 | 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| 无对外话题 | — | — | 演示结果仅通过 `RCLCPP_INFO` 日志输出 |

### 运动学库（kinematics_lib）

纯数学库，无 ROS 依赖，可独立被其他包链接使用：

| 类 | 方法 | 输入 | 输出 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `Planar2DOF` | `forward(JointState)` | `{theta1, theta2}`（弧度）| `{x, y}`（米）| 平面 2-DOF 正运动学 |
| `Planar2DOF` | `inverse(EndEffectorState)` | `{x, y}`（米）| `vector<JointState>`（0~2 组）| 几何法逆运动学，含工作空间检查 |
| `SphericalWrist6DOF` | `forward(JointState)` | `theta[6]`（弧度）| `{x, y, z, roll, pitch, yaw, R}` | D-H 矩阵连乘正运动学 |
| `SphericalWrist6DOF` | `inverse(EndEffectorState)` | `{x, y, z, R}`（位置+旋转矩阵）| `vector<JointState>` | 运动学解耦逆运动学（当前实现返回单解）|

## 验收测试

```bash
cd /root/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_kinematics
colcon test --packages-select ros2_learning_kinematics
colcon test-result --verbose
```

测试目标：

| 目标名 | 测试文件 | 用例数 | 覆盖内容 |
| :--- | :--- | :--- | :--- |
| `test_planar_2dof` | `test/test_planar_2dof.cpp` | 9 | FK 零位/单轴旋转/折叠，IK 往返精度、双解、单解、超出工作空间 |
| `test_spherical_wrist_6dof` | `test/test_spherical_wrist_6dof.cpp` | 6 | FK 零位、旋转矩阵正交性、欧拉角累加、IK 冒烟测试 |

测试精度要求：FK/IK 往返误差 < 1e-9 m（双精度浮点极限）。

## 已知限制

- `SphericalWrist6DOF` 中所有 6 个关节的 D-H 参数旋转轴均被简化为绕 Z 轴旋转（alpha 全为 0），导致末端位置 (x, y) 始终为 (0, 0)，无法表达真实六轴机械臂的三维工作空间。这是教学简化模型，不能直接移植到实体机器人。
- `SphericalWrist6DOF::inverse` 当前仅返回单解，未实现多解枚举和肘上/肘下选择。
- `ik_demo_node` 的随机数生成使用 `rand()` 且未设置随机种子，每次启动生成的序列相同，不适合覆盖性随机测试。
- 库依赖 Eigen3，未做版本兼容性检查；在 Eigen 3.4 以下可能存在 API 不兼容问题。

## 学习建议

1. 先看 `planar_2dof.cpp`：理解余弦定理是如何锁定关节角度的。
2. 再看 `spherical_wrist_6dof.cpp`：理解手腕中心点是如何让 6 轴计算变简单的。
3. 思考：为什么 6 自由度是三维世界的“黄金标准”？

---
> [!TIP]
> 机器人运动学的本质不是编程，而是**解析几何**。理解了 D-H 矩阵，你就拿到了通往高性能运动控制器的钥匙。
