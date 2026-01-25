#include "ros2_learning_arm_basics/arm_controller_params.hpp"

void ArmControllerParams::load(rclcpp::Node* node) {
    // 声明并获取参数
    // Links
    node->declare_parameter("links.base_frame", links.base_frame);
    node->declare_parameter("links.gripper_frame", links.gripper_frame);
    node->declare_parameter("links.target_box_id", links.target_box_id);
    node->declare_parameter("links.table_id", links.table_id);
    
    links.base_frame = node->get_parameter("links.base_frame").as_string();
    links.gripper_frame = node->get_parameter("links.gripper_frame").as_string();
    links.target_box_id = node->get_parameter("links.target_box_id").as_string();
    links.table_id = node->get_parameter("links.table_id").as_string();

    // Gripper
    node->declare_parameter("gripper.max_width", gripper.max_width);
    node->declare_parameter("gripper.open_width", gripper.open_width);
    node->declare_parameter("gripper.grasp_width", gripper.grasp_width);
    
    gripper.max_width = node->get_parameter("gripper.max_width").as_double();
    gripper.open_width = node->get_parameter("gripper.open_width").as_double();
    gripper.grasp_width = node->get_parameter("gripper.grasp_width").as_double();

    // Object
    node->declare_parameter("object.distance", object.distance);
    node->declare_parameter("object.box_size", object.box_size);
    // Table params
    node->declare_parameter("object.table_width", object.table_width);
    node->declare_parameter("object.table_depth", object.table_depth);
    node->declare_parameter("object.table_thickness", object.table_thickness);

    object.distance = node->get_parameter("object.distance").as_double();
    object.box_size = node->get_parameter("object.box_size").as_double();
    object.table_width = node->get_parameter("object.table_width").as_double();
    object.table_depth = node->get_parameter("object.table_depth").as_double();
    object.table_thickness = node->get_parameter("object.table_thickness").as_double();

    // Positions
    node->declare_parameter("positions.prepare_height", positions.prepare_height);
    node->declare_parameter("positions.grasp_height", positions.grasp_height);
    node->declare_parameter("positions.lift_height", positions.lift_height);
    node->declare_parameter("positions.place_height", positions.place_height);
    node->declare_parameter("positions.place_offset_y", positions.place_offset_y);

    positions.prepare_height = node->get_parameter("positions.prepare_height").as_double();
    positions.grasp_height = node->get_parameter("positions.grasp_height").as_double();
    positions.lift_height = node->get_parameter("positions.lift_height").as_double();
    positions.place_height = node->get_parameter("positions.place_height").as_double();
    positions.place_offset_y = node->get_parameter("positions.place_offset_y").as_double();
    
    // Grasp Orientation
    node->declare_parameter("orientation.x", grasp_orientation.x);
    node->declare_parameter("orientation.y", grasp_orientation.y);
    node->declare_parameter("orientation.z", grasp_orientation.z);
    node->declare_parameter("orientation.w", grasp_orientation.w);
    
    grasp_orientation.x = node->get_parameter("orientation.x").as_double();
    grasp_orientation.y = node->get_parameter("orientation.y").as_double();
    grasp_orientation.z = node->get_parameter("orientation.z").as_double();
    grasp_orientation.w = node->get_parameter("orientation.w").as_double();

    // Timing
    node->declare_parameter("timing.short_delay_ms", timing.short_delay_ms);
    node->declare_parameter("timing.medium_delay_ms", timing.medium_delay_ms);
    node->declare_parameter("timing.long_delay_ms", timing.long_delay_ms);
    node->declare_parameter("timing.scene_setup_delay_ms", timing.scene_setup_delay_ms);

    timing.short_delay_ms = node->get_parameter("timing.short_delay_ms").as_int();
    timing.medium_delay_ms = node->get_parameter("timing.medium_delay_ms").as_int();
    timing.long_delay_ms = node->get_parameter("timing.long_delay_ms").as_int();
    timing.scene_setup_delay_ms = node->get_parameter("timing.scene_setup_delay_ms").as_int();
    
    RCLCPP_INFO(node->get_logger(), "参数加载完成");
}
