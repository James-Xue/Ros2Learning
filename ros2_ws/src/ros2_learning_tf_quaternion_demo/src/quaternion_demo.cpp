// quaternion_demo.cpp
// 四元数演示节点 - 实现文件

#include "ros2_learning_tf_quaternion_demo/quaternion_demo.hpp"

#include <cmath>

QuaternionDemo::QuaternionDemo() 
    : Node("quaternion_demo"),
      m_demoStep(0),
      m_logger(this->get_logger())
{
    // 创建可视化发布器
    m_markerPub = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/quaternion_demo/markers", 10
    );
    
    // 创建定时器，每秒更新一次
    m_timer = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&QuaternionDemo::timerCallback, this)
    );
    
    RCLCPP_INFO(m_logger, "====================================");
    RCLCPP_INFO(m_logger, "  四元数演示节点已启动");
    RCLCPP_INFO(m_logger, "  在 RViz 中查看可视化效果");
    RCLCPP_INFO(m_logger, "====================================\n");
}

void QuaternionDemo::timerCallback() {
    visualization_msgs::msg::MarkerArray markers;
    
    // 根据演示步骤显示不同的四元数概念
    switch (m_demoStep % 6) {
        case 0:
            demonstrateNoRotation(markers);
            break;
        case 1:
            demonstrateZAxisRotation(markers);
            break;
        case 2:
            demonstrateXAxisRotation(markers);
            break;
        case 3:
            demonstrateYAxisRotation(markers);
            break;
        case 4:
            demonstrateEulerAngles(markers);
            break;
        case 5:
            demonstrateQuaternionMultiplication(markers);
            break;
    }
    
    m_markerPub->publish(markers);
    m_demoStep++;
}

void QuaternionDemo::demonstrateNoRotation(
    visualization_msgs::msg::MarkerArray& markers
) {
    RCLCPP_INFO(m_logger, "\n[演示 1/6] 单位四元数 - 无旋转");
    RCLCPP_INFO(m_logger, "四元数: (x=0, y=0, z=0, w=1)");
    RCLCPP_INFO(m_logger, "含义: 没有任何旋转");
    
    tf2::Quaternion q(0, 0, 0, 1);
    addCoordinateFrame(markers, 0, "identity", q, 
                      {0.0, 0.0, 0.0}, "无旋转（单位四元数）");
}

void QuaternionDemo::demonstrateZAxisRotation(
    visualization_msgs::msg::MarkerArray& markers
) {
    RCLCPP_INFO(m_logger, "\n[演示 2/6] 绕 Z 轴旋转 90°");
    
    double angle = M_PI / 2;
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    
    RCLCPP_INFO(m_logger, "四元数: (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                q.x(), q.y(), q.z(), q.w());
    RCLCPP_INFO(m_logger, "说明: z=sin(45°)=0.707, w=cos(45°)=0.707");
    RCLCPP_INFO(m_logger, "  angle/2 = 90°/2 = 45°");
    
    addCoordinateFrame(markers, 1, "z_rotation", q,
                      {2.0, 0.0, 0.0}, "绕Z轴旋转90°");
}

void QuaternionDemo::demonstrateXAxisRotation(
    visualization_msgs::msg::MarkerArray& markers
) {
    RCLCPP_INFO(m_logger, "\n[演示 3/6] 绕 X 轴旋转 90°");
    
    double angle = M_PI / 2;
    tf2::Quaternion q;
    q.setRPY(angle, 0, 0);
    
    RCLCPP_INFO(m_logger, "四元数: (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                q.x(), q.y(), q.z(), q.w());
    
    addCoordinateFrame(markers, 2, "x_rotation", q,
                      {0.0, 2.0, 0.0}, "绕X轴旋转90°");
}

void QuaternionDemo::demonstrateYAxisRotation(
    visualization_msgs::msg::MarkerArray& markers
) {
    RCLCPP_INFO(m_logger, "\n[演示 4/6] 绕 Y 轴旋转 90°");
    
    double angle = M_PI / 2;
    tf2::Quaternion q;
    q.setRPY(0, angle, 0);
    
    RCLCPP_INFO(m_logger, "四元数: (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                q.x(), q.y(), q.z(), q.w());
    
    addCoordinateFrame(markers, 3, "y_rotation", q,
                      {0.0, 0.0, 2.0}, "绕Y轴旋转90°");
}

void QuaternionDemo::demonstrateEulerAngles(
    visualization_msgs::msg::MarkerArray& markers
) {
    RCLCPP_INFO(m_logger, "\n[演示 5/6] 欧拉角 → 四元数");
    
    double roll = M_PI / 6;
    double pitch = M_PI / 4;
    double yaw = M_PI / 3;
    
    RCLCPP_INFO(m_logger, "欧拉角: roll=30°, pitch=45°, yaw=60°");
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(m_logger, "转换后的四元数:");
    RCLCPP_INFO(m_logger, "  x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                q.x(), q.y(), q.z(), q.w());
    
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    RCLCPP_INFO(m_logger, "验证（四元数→欧拉角）:");
    RCLCPP_INFO(m_logger, "  roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                r * 180.0 / M_PI, p * 180.0 / M_PI, y * 180.0 / M_PI);
    
    addCoordinateFrame(markers, 4, "euler", q,
                      {-2.0, 0.0, 0.0}, "复合旋转(30°,45°,60°)");
}

void QuaternionDemo::demonstrateQuaternionMultiplication(
    visualization_msgs::msg::MarkerArray& markers
) {
    RCLCPP_INFO(m_logger, "\n[演示 6/6] 四元数乘法 - 组合旋转");
    
    tf2::Quaternion q1;
    q1.setRPY(0, 0, M_PI / 4);
    RCLCPP_INFO(m_logger, "第一次旋转: 绕Z轴45°");
    
    tf2::Quaternion q2;
    q2.setRPY(M_PI / 4, 0, 0);
    RCLCPP_INFO(m_logger, "第二次旋转: 绕X轴45°");
    
    tf2::Quaternion q_combined = q2 * q1;
    
    RCLCPP_INFO(m_logger, "组合后的四元数:");
    RCLCPP_INFO(m_logger, "  x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                q_combined.x(), q_combined.y(), 
                q_combined.z(), q_combined.w());
    
    addCoordinateFrame(markers, 5, "combined", q_combined,
                      {0.0, -2.0, 0.0}, "组合旋转(Z45°+X45°)");
}

void QuaternionDemo::addCoordinateFrame(
    visualization_msgs::msg::MarkerArray& markers,
    int id_offset,
    const std::string& ns,
    const tf2::Quaternion& q,
    const std::array<double, 3>& position,
    const std::string& text
) {
    markers.markers.push_back(
        createAxisMarker(id_offset * 10 + 0, ns + "_x", q, position,
                       {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 1.0})
    );
    
    markers.markers.push_back(
        createAxisMarker(id_offset * 10 + 1, ns + "_y", q, position,
                       {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0, 1.0})
    );
    
    markers.markers.push_back(
        createAxisMarker(id_offset * 10 + 2, ns + "_z", q, position,
                       {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0, 1.0})
    );
    
    markers.markers.push_back(
        createTextMarker(id_offset * 10 + 3, ns + "_text", position, text)
    );
}

visualization_msgs::msg::Marker QuaternionDemo::createAxisMarker(
    int id,
    const std::string& ns,
    const tf2::Quaternion& q,
    const std::array<double, 3>& position,
    const std::array<double, 3>& /* direction */,  // 未使用但保留接口
    const std::array<double, 4>& color
) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation = tf2::toMsg(q);
    
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    
    return marker;
}

visualization_msgs::msg::Marker QuaternionDemo::createTextMarker(
    int id,
    const std::string& ns,
    const std::array<double, 3>& position,
    const std::string& text
) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2] + 0.5;
    
    marker.scale.z = 0.2;
    
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    marker.text = text;
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    
    return marker;
}
