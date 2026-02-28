# ros2_learning_tf_quaternion_demo

> 通过三个可交互演示节点和 RViz 可视化，讲解四元数（Quaternion）旋转表示与 ROS 2 TF 坐标变换系统的核心概念。

## 功能描述

本包包含三个独立的演示节点：

- **`quaternion_demo`**：每秒循环展示 6 种四元数概念（单位四元数、绕 X/Y/Z 轴旋转 90°、欧拉角转换、四元数乘法），通过向 `/quaternion_demo/markers` 发布 `MarkerArray` 在 RViz 中绘制带标注的坐标轴箭头。
- **`tf_broadcaster_demo`**：以 10 Hz 广播 4 层级 TF 树（`world` → `robot_base` → `rotating_platform`/`target_object` → `sensor_frame`），其中 `rotating_platform` 绕 Z 轴匀速旋转，`target_object` 沿圆弧轨道运动。
- **`tf_listener_demo`**：每秒查询并打印三组变换：传感器在世界坐标系中的位姿、传感器坐标系内一点转换到世界坐标系的结果、目标物体相对于传感器的距离。

## 运行命令

```bash
# 编译
cd /root/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_tf_quaternion_demo
source install/setup.bash

# 一键启动所有节点 + RViz
ros2 launch ros2_learning_tf_quaternion_demo demo.launch.py

# 单独运行各节点
ros2 run ros2_learning_tf_quaternion_demo quaternion_demo
ros2 run ros2_learning_tf_quaternion_demo tf_broadcaster_demo
ros2 run ros2_learning_tf_quaternion_demo tf_listener_demo

# 常用调试命令
ros2 run tf2_tools view_frames           # 生成 TF 树 PDF
ros2 run tf2_ros tf2_echo world sensor_frame  # 实时查看特定变换
```

## 输入/输出

### quaternion_demo 节点

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/quaternion_demo/markers` | `visualization_msgs/msg/MarkerArray` | 6 组坐标轴箭头和文字标注，`frame_id=world` |

### tf_broadcaster_demo 节点

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/tf` | `tf2_msgs/msg/TFMessage` | 动态 TF：`robot_base`、`rotating_platform`、`sensor_frame`、`target_object` |

> `world` → `map` 之间的静态 TF 由 launch 文件中的 `static_transform_publisher` 发布。

### tf_listener_demo 节点

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/tf` / `/tf_static` | `tf2_msgs/msg/TFMessage` | 通过 `tf2_ros::Buffer` 监听所有变换 |

### TF 树结构

```
map (static_transform_publisher)
 └─ world
     └─ robot_base
         ├─ rotating_platform  （绕 Z 轴旋转，10 Hz 更新）
         │   └─ sensor_frame   （固定偏移 +30° 俯仰）
         └─ target_object      （圆弧轨道运动）
```

## 验收测试

暂无，待补充。

## 已知限制

- `tf_listener_demo` 启动初期（约 1 秒内）会打印 `TransformException` 警告，属正常现象；TF 数据到达后自动恢复。
- `quaternion_demo` 每个演示帧的 Marker `lifetime` 设置为 2 秒，切换间隔为 1 秒；若 RViz 刷新率低，可能出现闪烁。
- RViz 配置依赖 `rviz/demo.rviz`，Fixed Frame 必须设置为 `world`；在无显示器的纯终端环境中 RViz 无法启动。

---

# ROS 2 四元数和 TF 变换演示

本包通过交互式可视化和详细注释的代码，帮助您彻底理解四元数（Quaternion）和坐标变换（TF）的概念。

## 🎯 学习目标

完成本演示后，您将掌握：
- ✅ 四元数的基本概念和数学原理
- ✅ 四元数与欧拉角的转换
- ✅ 四元数的运算（乘法、插值）
- ✅ ROS 2 TF 系统的使用
- ✅ 多层级坐标系的建立和查询
- ✅ 点和姿态的坐标转换

## 🚀 快速开始

### 1. 编译包

```bash
cd ~/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_tf_quaternion_demo
source install/setup.bash
```

### 2. 运行演示

```bash
ros2 launch ros2_learning_tf_quaternion_demo demo.launch.py
```

这个命令会启动：
- 🎨 **四元数可视化节点** - 展示 6 种四元数概念
- 📡 **TF 广播器** - 创建动态坐标系
- 📥 **TF 监听器** - 查询和转换坐标
- 🖥️ **RViz** - 3D 可视化

### 3. 观察和学习

**在 RViz 中**：
- 观察不同颜色的坐标轴（X=红，Y=绿，Z=蓝）
- 看旋转的坐标系如何变化
- 理解"无旋转"vs"90度旋转"的区别

**在终端中**：
- 阅读节点输出的详细解释
- 看四元数的数值变化
- 理解每个演示步骤的含义

## 📚 演示内容

### 演示 1: 四元数基础（共 6 个概念）

四元数演示节每秒切换，展示：

1. **单位四元数** (x=0, y=0, z=0, w=1)
   - 表示无旋转
   - 坐标轴保持原始方向

2. **绕 Z 轴旋转 90°**
   - z=0.707, w=0.707
   - X 轴指向原来的 Y 轴方向

3. **绕 X 轴旋转 90°**
   - x=0.707, w=0.707
   - Y 轴指向原来的 Z 轴方向

4. **绕 Y 轴旋转 90°**
   - y=0.707, w=0.707
   - Z 轴指向原来的 X 轴方向

5. **欧拉角转四元数**
   - roll=30°, pitch=45°, yaw=60°
   - 展示复合旋转

6. **四元数乘法**
   - 组合两次旋转
   - q_combined = q2 * q1

### 演示 2: TF 坐标系层级

```
world (静态)
  └─ robot_base
      ├─ rotating_platform (绕Z轴旋转)
      │   └─ sensor_frame (固定偏移，倾斜30°)
      └─ target_object (圆形轨道运动)
```

### 演示 3: TF 查询示例

TF 监听器每秒输出：
- 传感器在世界坐标系中的位姿
- 点的坐标转换（传感器→世界）
- 目标相对于传感器的距离

## 📖 核心概念解释

### 四元数是什么？

四元数是表示 3D 旋转的一种方法，由 4 个数组成：

```
q = (x, y, z, w)
```

**数学含义**：绕某个轴旋转一定角度
```
x = axis_x * sin(angle/2)
y = axis_y * sin(angle/2)
z = axis_z * sin(angle/2)
w = cos(angle/2)
```

**约束条件**：必须是单位四element数
```
x² + y² + z² + w² = 1
```

### 为什么使用四元数？

| 对比 | 欧拉角 | 四元数 |
|------|--------|--------|
| 直观性 | ✅ 容易理解 | ❌ 不直观 |
| 万向锁 | ❌ 会出现 | ✅ 无此问题 |
| 插值 | ❌ 不平滑 | ✅ 平滑 |
| 内存 | 3个数 | 4个数 |
| 计算 | 较慢 | 较快 |

**ROS/MoveIt 选择四元数的原因**：避免万向锁！

### TF 是什么？

TF (Transform) 是 ROS 中的坐标变换系统，用于：
- 📍 管理多个坐标系
- 🔄 自动计算坐标系之间的关系
- 🎯 转换点/姿态到不同坐标系

## 🧪 实践练习

### 练习 1：修改旋转角度

编辑 `src/quaternion_demo.cpp`，尝试不同的旋转角度：

```cpp
// 绕 Z 轴旋转 45° 而不是 90°
double angle = M_PI / 4;  // 45度
```

重新编译并运行，观察变化！

### 练习 2：添加新的坐标系

编辑 `src/tf_broadcaster_demo.cpp`，添加一个新的子坐标系：

```cpp
// 添加一个相机坐标系
geometry_msgs::msg::TransformStamped camera_tf;
camera_tf.header.stamp = now;
camera_tf.header.frame_id = "sensor_frame";
camera_tf.child_frame_id = "camera";
// ... 设置位置和方向
tf_broadcaster_->sendTransform(camera_tf);
```

### 练习 3：查询自定义变换

编辑 `src/tf_listener_demo.cpp`，查询您新添加的坐标系。

## 🔧 常用工具命令

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 实时查看特定变换
ros2 run tf2_ros tf2_echo world sensor_frame

# 查看所有 TF
ros2 topic echo /tf

# 查看 TF 统计
rqt_tf_tree
```

## 📐 常用代码模板

### 创建四元数

```cpp
#include <tf2/LinearMath/Quaternion.h>

tf2::Quaternion q;

// 方法1：从欧拉角
q.setRPY(roll, pitch, yaw);

// 方法2：从轴角
q.setRotation(tf2::Vector3(0, 0, 1), angle);  // 绕Z轴

// 方法3：直接设置
q = tf2::Quaternion(x, y, z, w);
```

### 四元数转欧拉角

```cpp
#include <tf2/LinearMath/Matrix3x3.h>

tf2::Quaternion q = ...;
double roll, pitch, yaw;
tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
```

### 广播 TF

```cpp
#include <tf2_ros/transform_broadcaster.h>

auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

geometry_msgs::msg::TransformStamped t;
t.header.stamp = node->now();
t.header.frame_id = "parent";
t.child_frame_id = "child";
// ... 设置 translation 和 rotation
tf_broadcaster->sendTransform(t);
```

### 查询 TF

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

try {
    auto transform = tf_buffer->lookupTransform(
        "target_frame",
        "source_frame",
        tf2::TimePointZero
    );
    // 使用 transform
} catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node->get_logger(), "%s", ex.what());
}
```

## 🎓 进阶资源

- [TF2 官方教程](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [四元数可视化工具](https://quaternions.online/)
- [MoveIt 教程](https://moveit.picknik.ai/jazzy/)

## 💡 调试技巧

1. **RViz 中看不到坐标轴？**
   - 检查 Fixed Frame 是否设置为 "world"
   - 添加 TF display 插件

2. **四元数值看起来很奇怪？**
   - 记住：角度需要除以 2！
   - angle/2 = 90°/2 = 45° → sin(45°) = 0.707

3. **TF 查询失败？**
   - 确保父子坐标系都被广播
   - 检查时间戳是否正确

## 📝 总结

**四元数**：
- 用 4 个数表示 3D 旋转
- w=1 表示无旋转
- 避免了万向锁问题

**TF**：
- 管理多个坐标系
- 自动计算变换
- 支持动态和静态变换

现在开始运行演示，亲自体验吧！🚀
