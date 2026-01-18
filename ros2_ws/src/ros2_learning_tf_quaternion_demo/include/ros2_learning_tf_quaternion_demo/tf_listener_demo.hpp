// tf_listener_demo.hpp
// TF 监听器演示节点 - 头文件

#ifndef ROS2_LEARNING_TF_QUATERNION_DEMO_TF_LISTENER_DEMO_HPP_
#define ROS2_LEARNING_TF_QUATERNION_DEMO_TF_LISTENER_DEMO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <memory>

/**
 * @class TFListenerDemo
 * @brief TF 监听器演示 - 查询和使用坐标变换
 * 
 * 本节点演示：
 * 1. 查询坐标系之间的变换
 * 2. 转换点的坐标
 * 3. 计算相对位置和距离
 */
class TFListenerDemo : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    TFListenerDemo();

private:
    /**
     * @brief 查询各种TF变换
     */
    void lookupTransforms();
    
    /// TF buffer（存储变换历史）
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    
    /// TF listener（接收变换）
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    
    /// 定时器(1Hz)
    rclcpp::TimerBase::SharedPtr m_timer;
    
    /// 日志记录器
    rclcpp::Logger m_logger;
};

#endif  // ROS2_LEARNING_TF_QUATERNION_DEMO_TF_LISTENER_DEMO_HPP_
