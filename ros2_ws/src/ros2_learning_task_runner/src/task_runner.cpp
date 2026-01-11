#include "task_runner.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <exception>

using namespace std::chrono_literals;

TaskRunner::TaskRunner()
        : Node("task_runner")
{
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    nav2_action_name_ = this->declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
    task_config_path_ = this->declare_parameter<std::string>("task_config", "");
    pick_service_name_ = this->declare_parameter<std::string>("pick_service", "/manipulation/pick");
    place_service_name_ = this->declare_parameter<std::string>("place_service", "/manipulation/place");
    clock_wait_timeout_sec_ = this->declare_parameter<double>("clock_wait_timeout_sec", 20.0);
    navigation_timeout_sec_ = this->declare_parameter<double>("navigation_timeout_sec", 120.0);
    use_sim_time_ = this->declare_parameter<bool>("use_sim_time", true);

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
    pick_client_ = this->create_client<Trigger>(pick_service_name_);
    place_client_ = this->create_client<Trigger>(place_service_name_);

    distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance_remaining", 10);
}

void TaskRunner::run()
{
    if (use_sim_time_)
    {
        RCLCPP_INFO(get_logger(), "use_sim_time=true, waiting for /clock...");
        if (!wait_for_time())
        {
            RCLCPP_ERROR(get_logger(), "/clock not received within %.1f seconds.", clock_wait_timeout_sec_);
            return;
        }
    }

    if (!load_task_config(task_config_path_))
    {
        RCLCPP_ERROR(get_logger(), "Failed to load task config: '%s'", task_config_path_.c_str());
        return;
    }

    if (!wait_for_action_server())
    {
        RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable.");
        return;
    }

    if (!wait_for_service(pick_client_, pick_service_name_))
    {
        RCLCPP_ERROR(get_logger(), "Pick service unavailable: %s", pick_service_name_.c_str());
        return;
    }

    if (!wait_for_service(place_client_, place_service_name_))
    {
        RCLCPP_ERROR(get_logger(), "Place service unavailable: %s", place_service_name_.c_str());
        return;
    }

    if (!has_dropoff_ || pickup_points_.empty())
    {
        RCLCPP_ERROR(get_logger(), "Task config missing dropoff or pickups.");
        return;
    }

    for (size_t i = 0; rclcpp::ok() && i < pickup_points_.size(); ++i)
    {
        const auto &pickup = pickup_points_[i];
        RCLCPP_INFO(get_logger(), "Pickup %zu/%zu: going to (%.2f, %.2f, %.2f)",
                    i + 1, pickup_points_.size(), pickup.x, pickup.y, pickup.yaw);

        if (!navigate_to(pickup, "pickup"))
        {
            RCLCPP_WARN(get_logger(), "Navigation to pickup %zu failed, skipping.", i + 1);
            continue;
        }

        if (!call_trigger(pick_client_, "pick"))
        {
            RCLCPP_WARN(get_logger(), "Pick failed at pickup %zu, skipping dropoff.", i + 1);
            continue;
        }

        RCLCPP_INFO(get_logger(), "Going to dropoff (%.2f, %.2f, %.2f)",
                    dropoff_.x, dropoff_.y, dropoff_.yaw);
        if (!navigate_to(dropoff_, "dropoff"))
        {
            RCLCPP_WARN(get_logger(), "Navigation to dropoff failed, skipping place.");
            continue;
        }

        if (!call_trigger(place_client_, "place"))
        {
            RCLCPP_WARN(get_logger(), "Place failed after pickup %zu.", i + 1);
            continue;
        }
    }

    RCLCPP_INFO(get_logger(), "Task runner completed.");
}

bool TaskRunner::load_task_config(const std::string &path)
{
    if (path.empty())
    {
        RCLCPP_ERROR(get_logger(), "Parameter 'task_config' is empty.");
        return false;
    }

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "YAML load error: %s", e.what());
        return false;
    }

    if (root["map_frame"])
    {
        map_frame_ = root["map_frame"].as<std::string>();
    }

    if (!parse_pose_node(root["dropoff"], "dropoff", &dropoff_))
    {
        return false;
    }
    has_dropoff_ = true;

    pickup_points_.clear();
    const auto pickups = root["pickups"];
    if (!pickups || !pickups.IsSequence())
    {
        RCLCPP_ERROR(get_logger(), "'pickups' must be a list in task config.");
        return false;
    }

    for (size_t i = 0; i < pickups.size(); ++i)
    {
        Pose2D pose;
        if (!parse_pose_node(pickups[i], "pickup", &pose))
        {
            RCLCPP_ERROR(get_logger(), "Invalid pickup at index %zu.", i);
            return false;
        }
        pickup_points_.push_back(pose);
    }

    return true;
}

bool TaskRunner::parse_pose_node(const YAML::Node &node, const std::string &label, Pose2D *out_pose)
{
    if (!node || !out_pose)
    {
        RCLCPP_ERROR(get_logger(), "Missing %s pose in task config.", label.c_str());
        return false;
    }

    if (!node["x"] || !node["y"])
    {
        RCLCPP_ERROR(get_logger(), "%s pose requires 'x' and 'y'.", label.c_str());
        return false;
    }

    out_pose->x = node["x"].as<double>();
    out_pose->y = node["y"].as<double>();
    out_pose->yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
    return true;
}

bool TaskRunner::wait_for_time()
{
    const auto start_wall = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        const auto now_ns = this->get_clock()->now().nanoseconds();
        if (now_ns != 0)
        {
            return true;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start_wall;
        if (std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count() > clock_wait_timeout_sec_)
        {
            return false;
        }

        rclcpp::sleep_for(100ms);
    }

    return false;
}

bool TaskRunner::wait_for_action_server()
{
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server '%s'...", nav2_action_name_.c_str());
    return action_client_->wait_for_action_server(10s);
}

bool TaskRunner::wait_for_service(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name)
{
    RCLCPP_INFO(get_logger(), "Waiting for service '%s'...", name.c_str());
    return client->wait_for_service(10s);
}

TaskRunner::PoseStamped TaskRunner::make_pose(const Pose2D &pose) const
{
    PoseStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->now();
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    msg.pose.orientation = tf2::toMsg(q);

    return msg;
}

bool TaskRunner::navigate_to(const Pose2D &pose, const std::string &label)
{
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = make_pose(pose);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
        [this, label](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        std_msgs::msg::Float32 msg;
        msg.data = feedback->distance_remaining;
        distance_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[%s] remaining distance: %.2f", label.c_str(), feedback->distance_remaining);
    };

    auto goal_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    const auto send_code = rclcpp::spin_until_future_complete(shared_from_this(), goal_future, 5s);
    if (send_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Send goal failed for %s.", label.c_str());
        return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal rejected for %s.", label.c_str());
        return false;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    const std::chrono::duration<double> timeout(navigation_timeout_sec_);
    const auto result_code = rclcpp::spin_until_future_complete(shared_from_this(), result_future, timeout);

    if (result_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        const auto wrapped_result = result_future.get();
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Navigation to %s succeeded.", label.c_str());
            return true;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Navigation to %s aborted.", label.c_str());
            return false;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Navigation to %s canceled.", label.c_str());
            return false;
        default:
            RCLCPP_WARN(get_logger(), "Navigation to %s returned unknown result.", label.c_str());
            return false;
        }
    }

    RCLCPP_WARN(get_logger(), "Navigation to %s timed out, canceling goal.", label.c_str());
    auto cancel_future = action_client_->async_cancel_goal(goal_handle);
    (void)rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 5s);
    return false;
}

bool TaskRunner::call_trigger(const rclcpp::Client<Trigger>::SharedPtr &client, const std::string &name)
{
    auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);
    const auto result_code = rclcpp::spin_until_future_complete(shared_from_this(), future, 10s);
    if (result_code != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s service call failed.", name.c_str());
        return false;
    }

    const auto response = future.get();
    if (!response->success)
    {
        RCLCPP_WARN(get_logger(), "%s service returned failure: %s", name.c_str(), response->message.c_str());
    }

    return response->success;
}
