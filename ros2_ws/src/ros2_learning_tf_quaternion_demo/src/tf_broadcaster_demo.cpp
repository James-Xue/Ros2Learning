// tf_broadcaster_demo.cpp
// TF 广播器演示节点 - 实现文件

#include "ros2_learning_tf_quaternion_demo/tf_broadcaster_demo.hpp"

#include <cmath>

TFBroadcasterDemo::TFBroadcasterDemo() 
    : Node("tf_broadcaster_demo"),
      m_angle(0.0),
      m_logger(this->get_logger())
{
    // 创建 TF 广播器
    m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // 创建定时器，以 10Hz 更新 TF
    m_timer = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TFBroadcasterDemo::broadcastTF, this)
    );
    
    RCLCPP_INFO(m_logger, "====================================");
    RCLCPP_INFO(m_logger, "  TF 广播器演示节点已启动");
    RCLCPP_INFO(m_logger, "  广播多层级坐标系");
    RCLCPP_INFO(m_logger, "====================================\n");
}

void TFBroadcasterDemo::broadcastTF() {
    auto now = this->get_clock()->now();
    
    // ═══════════════════════════════════════════════════════
    // 1. 静态世界坐标系 → 机器人基座
    // ═══════════════════════════════════════════════════════
    geometry_msgs::msg::TransformStamped robot_base_tf;
    robot_base_tf.header.stamp = now;
    robot_base_tf.header.frame_id = "world";
    robot_base_tf.child_frame_id = "robot_base";
    
    robot_base_tf.transform.translation.x = 0.0;
    robot_base_tf.transform.translation.y = 0.0;
    robot_base_tf.transform.translation.z = 0.0;
    
    robot_base_tf.transform.rotation.x = 0.0;
    robot_base_tf.transform.rotation.y = 0.0;
    robot_base_tf.transform.rotation.z = 0.0;
    robot_base_tf.transform.rotation.w = 1.0;
    
    m_tfBroadcaster->sendTransform(robot_base_tf);
    
    // ═══════════════════════════════════════════════════════
    // 2. 机器人基座 → 旋转平台（动态旋转）
    // ═══════════════════════════════════════════════════════
    geometry_msgs::msg::TransformStamped rotating_platform_tf;
    rotating_platform_tf.header.stamp = now;
    rotating_platform_tf.header.frame_id = "robot_base";
    rotating_platform_tf.child_frame_id = "rotating_platform";
    
    rotating_platform_tf.transform.translation.x = 0.0;
    rotating_platform_tf.transform.translation.y = 0.0;
    rotating_platform_tf.transform.translation.z = 0.5;
    
    tf2::Quaternion q_rotate;
    q_rotate.setRPY(0, 0, m_angle);
    rotating_platform_tf.transform.rotation.x = q_rotate.x();
    rotating_platform_tf.transform.rotation.y = q_rotate.y();
    rotating_platform_tf.transform.rotation.z = q_rotate.z();
    rotating_platform_tf.transform.rotation.w = q_rotate.w();
    
    m_tfBroadcaster->sendTransform(rotating_platform_tf);
    
    // ═══════════════════════════════════════════════════════
    // 3. 旋转平台 → 传感器（固定偏移）
    // ═══════════════════════════════════════════════════════
    geometry_msgs::msg::TransformStamped sensor_tf;
    sensor_tf.header.stamp = now;
    sensor_tf.header.frame_id = "rotating_platform";
    sensor_tf.child_frame_id = "sensor_frame";
    
    sensor_tf.transform.translation.x = 1.0;
    sensor_tf.transform.translation.y = 0.0;
    sensor_tf.transform.translation.z = 0.2;
    
    tf2::Quaternion q_sensor;
    q_sensor.setRPY(0, M_PI/6, 0);
    sensor_tf.transform.rotation.x = q_sensor.x();
    sensor_tf.transform.rotation.y = q_sensor.y();
    sensor_tf.transform.rotation.z = q_sensor.z();
    sensor_tf.transform.rotation.w = q_sensor.w();
    
    m_tfBroadcaster->sendTransform(sensor_tf);
    
    // ═══════════════════════════════════════════════════════
    // 4. 机器人基座 → 目标物体（动态移动+旋转）
    // ═══════════════════════════════════════════════════════
    geometry_msgs::msg::TransformStamped target_tf;
    target_tf.header.stamp = now;
    target_tf.header.frame_id = "robot_base";
    target_tf.child_frame_id = "target_object";
    
    double orbit_radius = 2.0;
    target_tf.transform.translation.x = orbit_radius * cos(m_angle * 0.5);
    target_tf.transform.translation.y = orbit_radius * sin(m_angle * 0.5);
    target_tf.transform.translation.z = 1.0;
    
    tf2::Quaternion q_target;
    q_target.setRPY(m_angle * 0.3, m_angle * 0.2, m_angle);
    target_tf.transform.rotation.x = q_target.x();
    target_tf.transform.rotation.y = q_target.y();
    target_tf.transform.rotation.z = q_target.z();
    target_tf.transform.rotation.w = q_target.w();
    
    m_tfBroadcaster->sendTransform(target_tf);
    
    // 更新角度
    m_angle += 0.05;
    if (m_angle > 2 * M_PI) {
        m_angle -= 2 * M_PI;
    }
    
    // 每秒打印一次状态
    static int count = 0;
    if (count++ % 10 == 0) {
        RCLCPP_INFO(m_logger, "当前角度: %.1f°, 目标位置: (%.2f, %.2f, %.2f)",
                    m_angle * 180.0 / M_PI,
                    target_tf.transform.translation.x,
                    target_tf.transform.translation.y,
                    target_tf.transform.translation.z);
    }
}
