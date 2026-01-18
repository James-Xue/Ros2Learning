// quaternion_demo.hpp
// 四元数演示节点 - 头文件

#ifndef ROS2_LEARNING_TF_QUATERNION_DEMO_QUATERNION_DEMO_HPP_
#define ROS2_LEARNING_TF_QUATERNION_DEMO_QUATERNION_DEMO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <string>

/**
 * @class QuaternionDemo
 * @brief 四元数演示节点 - 通过RViz可视化理解四元数
 * 
 * 本节点循环展示6种四元数概念：
 * 1. 单位四元数（无旋转）
 * 2. 绕Z轴旋转90°
 * 3. 绕X轴旋转90°
 * 4. 绕Y轴旋转90°
 * 5. 欧拉角转四元数
 * 6. 四元数乘法（组合旋转）
 */
class QuaternionDemo : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    QuaternionDemo();

private:
    /**
     * @brief 定时器回调 - 切换演示步骤
     */
    void timerCallback();
    
    /**
     * @brief 演示1: 单位四元数（无旋转）
     */
    void demonstrateNoRotation(visualization_msgs::msg::MarkerArray& markers);
    
    /**
     * @brief 演示2: 绕Z轴旋转90°
     */
    void demonstrateZAxisRotation(visualization_msgs::msg::MarkerArray& markers);
    
    /**
     * @brief 演示3: 绕X轴旋转90°
     */
    void demonstrateXAxisRotation(visualization_msgs::msg::MarkerArray& markers);
    
    /**
     * @brief 演示4: 绕Y轴旋转90°
     */
    void demonstrateYAxisRotation(visualization_msgs::msg::MarkerArray& markers);
    
    /**
     * @brief 演示5: 欧拉角转四元数
     */
    void demonstrateEulerAngles(visualization_msgs::msg::MarkerArray& markers);
    
    /**
     * @brief 演示6: 四元数乘法（组合旋转）
     */
    void demonstrateQuaternionMultiplication(visualization_msgs::msg::MarkerArray& markers);
    
    /**
     * @brief 添加坐标系可视化标记
     */
    void addCoordinateFrame(
        visualization_msgs::msg::MarkerArray& markers,
        int id_offset,
        const std::string& ns,
        const tf2::Quaternion& q,
        const std::array<double, 3>& position,
        const std::string& text
    );
    
    /**
     * @brief 创建坐标轴箭头标记
     */
    visualization_msgs::msg::Marker createAxisMarker(
        int id,
        const std::string& ns,
        const tf2::Quaternion& q,
        const std::array<double, 3>& position,
        const std::array<double, 3>& direction,
        const std::array<double, 4>& color
    );
    
    /**
     * @brief 创建文本标记
     */
    visualization_msgs::msg::Marker createTextMarker(
        int id,
        const std::string& ns,
        const std::array<double, 3>& position,
        const std::string& text
    );
    
    /// 可视化标记发布器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markerPub;
    
    /// 定时器
    rclcpp::TimerBase::SharedPtr m_timer;
    
    /// 演示步骤计数器
    int m_demoStep;
    
    /// 日志记录器
    rclcpp::Logger m_logger;
};

#endif  // ROS2_LEARNING_TF_QUATERNION_DEMO_QUATERNION_DEMO_HPP_
