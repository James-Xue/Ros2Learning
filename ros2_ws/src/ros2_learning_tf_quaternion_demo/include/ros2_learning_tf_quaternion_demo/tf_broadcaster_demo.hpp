// tf_broadcaster_demo.hpp
// TF 广播器演示节点 - 头文件

#ifndef ROS2_LEARNING_TF_QUATERNION_DEMO_TF_BROADCASTER_DEMO_HPP_
#define ROS2_LEARNING_TF_QUATERNION_DEMO_TF_BROADCASTER_DEMO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>

/**
 * @class TFBroadcasterDemo
 * @brief TF 广播器演示 - 展示多层级动态坐标变换
 * 
 * 本节点创建以下坐标系层级：
 * world -> robot_base -> rotating_platform -> sensor_frame
 *                     -> target_object
 */
class TFBroadcasterDemo : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    TFBroadcasterDemo();

private:
    /**
     * @brief 广播所有TF变换
     */
    void broadcastTF();
    
    /// TF 广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
    
    /// 定时器(10Hz)
    rclcpp::TimerBase::SharedPtr m_timer;
    
    /// 当前旋转角度
    double m_angle;
    
    /// 日志记录器
    rclcpp::Logger m_logger;
};

#endif  // ROS2_LEARNING_TF_QUATERNION_DEMO_TF_BROADCASTER_DEMO_HPP_
