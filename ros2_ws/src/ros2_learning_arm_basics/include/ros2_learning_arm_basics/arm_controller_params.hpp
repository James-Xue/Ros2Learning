#ifndef ROS2_LEARNING_ARM_BASICS_ARM_CONTROLLER_PARAMS_HPP_
#define ROS2_LEARNING_ARM_BASICS_ARM_CONTROLLER_PARAMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

/**
 * @brief 机械臂控制器参数类
 * 
 * 集中管理所有可配置参数，支持从参数服务器加载
 */
struct ArmControllerParams {
    struct Links {
        std::string base_frame = "panda_link0";
        std::string gripper_frame = "panda_hand";
        std::string target_box_id = "target_box";
        std::string table_id = "table";
    } links;

    struct Gripper {
        double max_width = 0.08;
        double open_width = 0.035;
        double grasp_width = 0.03;
    } gripper;

    struct Object {
        double distance = 0.4;
        double box_size = 0.05;
        double table_width = 0.6;
        double table_depth = 0.8;
        double table_thickness = 0.02;
    } object;

    struct Positions {
        double prepare_height = 0.35;
        double grasp_height = 0.13;
        double lift_height = 0.5;
        double place_height = 0.3;
        double place_offset_y = -0.3;
    } positions;
    
    struct GraspOrientation {
        double x = 1.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
    } grasp_orientation;

    struct Timing {
        int short_delay_ms = 300;
        int medium_delay_ms = 500;
        int long_delay_ms = 1000;
        int scene_setup_delay_ms = 2000;
    } timing;

    /**
     * @brief 从ROS参数服务器加载参数
     * @param node ROS节点指针
     */
    void load(rclcpp::Node* node);
};

#endif // ROS2_LEARNING_ARM_BASICS_ARM_CONTROLLER_PARAMS_HPP_
