#include "ros2_learning_turtlesim_go_to/turtlesim_go_to_server_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <thread>

namespace ros2_learning_turtlesim_go_to
{

using namespace std::chrono_literals;

TurtlesimGoToServerNode::TurtlesimGoToServerNode()
        : rclcpp::Node("turtlesim_go_to_server")
{
    go_to_service_name_ = this->declare_parameter<std::string>(
        "service_name", "/ros2_learning/turtlesim/go_to");

    k_linear_ = this->declare_parameter<double>("k_linear", 1.5);
    k_angular_ = this->declare_parameter<double>("k_angular", 6.0);
    max_linear_ = this->declare_parameter<double>("max_linear", 2.0);
    max_angular_ = this->declare_parameter<double>("max_angular", 6.0);
    dist_tolerance_ = this->declare_parameter<double>("dist_tolerance", 0.05);
    theta_tolerance_ = this->declare_parameter<double>("theta_tolerance", 0.05);
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 30.0);
    wait_pose_timeout_sec_ =
        this->declare_parameter<double>("wait_pose_timeout_sec", 1.0);
    overall_timeout_sec_ =
        this->declare_parameter<double>("overall_timeout_sec", 10.0);

    service_ = this->create_service<GoTo>(
        go_to_service_name_,
        std::bind(&TurtlesimGoToServerNode::handle_go_to, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "GoTo service ready: %s",
                go_to_service_name_.c_str());
}

namespace
{
double clamp(double v, double lo, double hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

double normalize_angle(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

std::string pose_topic_for(const std::string &turtle)
{
    return "/" + turtle + "/pose";
}

std::string cmd_vel_topic_for(const std::string &turtle)
{
    return "/" + turtle + "/cmd_vel";
}
} // namespace

void TurtlesimGoToServerNode::ensure_io_for_turtle(const std::string &turtle)
{
    if (turtle == current_turtle_ && cmd_pub_ && pose_sub_)
    {
        return;
    }

    current_turtle_ = turtle;
    pose_received_.store(false);

    cmd_pub_ = this->create_publisher<Twist>(cmd_vel_topic_for(turtle), 10);
    pose_sub_ = this->create_subscription<Pose>(
        pose_topic_for(turtle), 10,
        [this](const Pose &msg)
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            last_pose_ = msg;
            pose_received_.store(true);
        });
}

void TurtlesimGoToServerNode::handle_go_to(
    const std::shared_ptr<GoTo::Request> request,
    std::shared_ptr<GoTo::Response> response)
{
    // Smooth motion controller:
    // - subscribe /<turtle>/pose
    // - publish /<turtle>/cmd_vel
    // - run a small P-controller loop until reaching target

    const std::string turtle =
        request->turtle_name.empty() ? "turtle1" : request->turtle_name;

    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true))
    {
        response->success = false;
        response->message = "server is busy (another goal is running)";
        return;
    }

    auto clear_busy = std::unique_ptr<void, std::function<void(void *)>>(
        nullptr, [this](void *) { busy_ = false; });

    ensure_io_for_turtle(turtle);

    // Wait until we have at least one pose sample.
    const auto wait_pose_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(wait_pose_timeout_sec_);
    while (rclcpp::ok() && !pose_received_.load() &&
           std::chrono::steady_clock::now() < wait_pose_deadline)
    {
        std::this_thread::sleep_for(10ms);
    }
    if (!pose_received_.load())
    {
        response->success = false;
        response->message =
            "no pose received on " + pose_topic_for(turtle) +
            ". Did you start: ros2 run turtlesim turtlesim_node ?";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    const double goal_x = request->x;
    const double goal_y = request->y;
    const double goal_theta = request->theta;

    RCLCPP_INFO(get_logger(), "Moving %s smoothly to (%.3f, %.3f, %.3f)",
                turtle.c_str(), goal_x, goal_y, goal_theta);

    rclcpp::Rate rate(control_rate_hz_ > 1e-3 ? control_rate_hz_ : 30.0);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(overall_timeout_sec_);

    enum class Stage
    {
        GoToXY,
        Rotate,
    };
    Stage stage = Stage::GoToXY;

    auto publish_stop = [this]()
    {
        Twist t;
        t.linear.x = 0.0;
        t.angular.z = 0.0;
        if (cmd_pub_)
        {
            cmd_pub_->publish(t);
        }
    };

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
        Pose pose;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            pose = last_pose_;
        }

        const double dx = goal_x - pose.x;
        const double dy = goal_y - pose.y;
        const double dist = std::hypot(dx, dy);

        Twist cmd;
        if (stage == Stage::GoToXY)
        {
            if (dist <= dist_tolerance_)
            {
                stage = Stage::Rotate;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
            else
            {
                const double desired_heading = std::atan2(dy, dx);
                const double heading_err =
                    normalize_angle(desired_heading - pose.theta);

                double linear = k_linear_ * dist;
                if (std::abs(heading_err) > 1.0)
                {
                    // If we face far away from the goal, rotate in-place first.
                    linear = 0.0;
                }
                linear = clamp(linear, 0.0, max_linear_);

                double angular = clamp(k_angular_ * heading_err, -max_angular_,
                                       max_angular_);

                cmd.linear.x = linear;
                cmd.angular.z = angular;
            }
        }
        else
        {
            const double theta_err = normalize_angle(goal_theta - pose.theta);
            if (std::abs(theta_err) <= theta_tolerance_)
            {
                publish_stop();
                response->success = true;
                std::ostringstream oss;
                oss << "ok: reached (" << goal_x << ", " << goal_y << ", "
                    << goal_theta << ")";
                response->message = oss.str();
                return;
            }
            cmd.linear.x = 0.0;
            cmd.angular.z =
                clamp(k_angular_ * theta_err, -max_angular_, max_angular_);
        }

        if (cmd_pub_)
        {
            cmd_pub_->publish(cmd);
        }
        rate.sleep();
    }

    publish_stop();
    response->success = false;
    response->message = "timeout reaching goal";
    return;
}

} // namespace ros2_learning_turtlesim_go_to
