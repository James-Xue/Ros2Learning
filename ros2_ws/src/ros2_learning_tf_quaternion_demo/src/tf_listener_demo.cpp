// tf_listener_demo.cpp
// TF 监听器演示节点 - 实现文件

#include "ros2_learning_tf_quaternion_demo/tf_listener_demo.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

TFListenerDemo::TFListenerDemo() 
    : Node("tf_listener_demo"),
      m_logger(this->get_logger())
{
    // 创建 TF buffer 和 listener
    m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
    
    // 创建定时器，每秒查询一次变换
    m_timer = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TFListenerDemo::lookupTransforms, this)
    );
    
    RCLCPP_INFO(m_logger, "====================================");
    RCLCPP_INFO(m_logger, "  TF 监听器演示节点已启动");
    RCLCPP_INFO(m_logger, "  正在监听坐标变换...");
    RCLCPP_INFO(m_logger, "====================================\n");
}

void TFListenerDemo::lookupTransforms() {
    // ═══════════════════════════════════════════════════════
    // 1. 查询：传感器相对于世界坐标系的位置
    // ═══════════════════════════════════════════════════════
    try {
        geometry_msgs::msg::TransformStamped transform_stamped =
            m_tfBuffer->lookupTransform(
                "world",
                "sensor_frame",
                tf2::TimePointZero
            );
        
        RCLCPP_INFO(m_logger, "\n[查询 1] 传感器在世界坐标系中的位姿:");
        RCLCPP_INFO(m_logger, "  位置: (%.3f, %.3f, %.3f)",
                    transform_stamped.transform.translation.x,
                    transform_stamped.transform.translation.y,
                    transform_stamped.transform.translation.z);
        RCLCPP_INFO(m_logger, "  方向: (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w);
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(m_logger, "无法查询变换: %s", ex.what());
    }
    
    // ═══════════════════════════════════════════════════════
    // 2. 点的坐标变换
    // ═══════════════════════════════════════════════════════
    try {
        geometry_msgs::msg::PointStamped point_in_sensor;
        point_in_sensor.header.frame_id = "sensor_frame";
        point_in_sensor.header.stamp = rclcpp::Time(0);  // 使用最新可用的 TF 数据
        point_in_sensor.point.x = 1.0;
        point_in_sensor.point.y = 0.0;
        point_in_sensor.point.z = 0.0;
        
        geometry_msgs::msg::PointStamped point_in_world =
            m_tfBuffer->transform(point_in_sensor, "world");
        
        RCLCPP_INFO(m_logger, "\n[查询 2] 点的坐标转换:");
        RCLCPP_INFO(m_logger, "  传感器坐标系: (%.3f, %.3f, %.3f)",
                    point_in_sensor.point.x,
                    point_in_sensor.point.y,
                    point_in_sensor.point.z);
        RCLCPP_INFO(m_logger, "  世界坐标系:   (%.3f, %.3f, %.3f)",
                    point_in_world.point.x,
                    point_in_world.point.y,
                    point_in_world.point.z);
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(m_logger, "点变换失败: %s", ex.what());
    }
    
    // ═══════════════════════════════════════════════════════
    // 3. 查询目标物体相对于传感器的位置
    // ═══════════════════════════════════════════════════════
    try {
        geometry_msgs::msg::TransformStamped sensor_to_target =
            m_tfBuffer->lookupTransform(
                "sensor_frame",
                "target_object",
                tf2::TimePointZero
            );
        
        double distance = std::sqrt(
            sensor_to_target.transform.translation.x * 
            sensor_to_target.transform.translation.x +
            sensor_to_target.transform.translation.y * 
            sensor_to_target.transform.translation.y +
            sensor_to_target.transform.translation.z * 
            sensor_to_target.transform.translation.z
        );
        
        RCLCPP_INFO(m_logger, "\n[查询 3] 目标相对于传感器:");
        RCLCPP_INFO(m_logger, "  相对位置: (%.3f, %.3f, %.3f)",
                    sensor_to_target.transform.translation.x,
                    sensor_to_target.transform.translation.y,
                    sensor_to_target.transform.translation.z);
        RCLCPP_INFO(m_logger, "  距离: %.3f 米", distance);
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(m_logger, "查询失败: %s", ex.what());
    }
}
