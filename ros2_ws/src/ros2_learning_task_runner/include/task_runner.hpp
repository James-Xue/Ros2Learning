#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

class TaskRunner : public rclcpp::Node
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using Trigger = std_srvs::srv::Trigger;

    TaskRunner();

    void run();

  private:
    struct Pose2D
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    bool load_task_config(const std::string &path);
    bool parse_pose_node(const YAML::Node &node, const std::string &label, Pose2D *out_pose);

    bool wait_for_time();
    bool wait_for_action_server();
    bool wait_for_service(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name);

    PoseStamped make_pose(const Pose2D &pose) const;
    bool navigate_to(const Pose2D &pose, const std::string &label);
    bool call_trigger(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name);

    std::string map_frame_;
    std::string nav2_action_name_;
    std::string task_config_path_;
    std::string pick_service_name_;
    std::string place_service_name_;
    double clock_wait_timeout_sec_{20.0};
    double navigation_timeout_sec_{120.0};
    bool use_sim_time_{true};

    Pose2D dropoff_;
    bool has_dropoff_{false};
    std::vector<Pose2D> pickup_points_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Client<Trigger>::SharedPtr pick_client_;
    rclcpp::Client<Trigger>::SharedPtr place_client_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
};
