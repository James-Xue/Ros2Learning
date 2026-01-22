#include "tb3_spawner_node.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace ros2_learning_tb3_spawner
{

using namespace std::chrono_literals;

Tb3SpawnerNode::Tb3SpawnerNode()
        : Node("tb3_spawner")
{
    // Defaults are tuned for the common tutorial flow:
    // - user starts Gazebo via turtlebot3_gazebo
    // - then starts this spawner node
    gazebo_spawn_service_ = this->declare_parameter<std::string>(
        "gazebo_spawn_service", "/world/default/create");

    const char *env_model = std::getenv("TURTLEBOT3_MODEL");
    default_model_ = this->declare_parameter<std::string>(
        "default_model", env_model ? std::string(env_model) : "waffle_pi");

    request_timeout_sec_ =
        this->declare_parameter<double>("request_timeout_sec", 5.0);

    spawn_client_ = this->create_client<SpawnEntity>(gazebo_spawn_service_);

    // Service callback may block while waiting for Gazebo response.
    // MultiThreadedExecutor in main keeps the client response flowing.
    spawn_service_ = this->create_service<SpawnTb3>(
        "spawn_tb3", std::bind(&Tb3SpawnerNode::handle_spawn, this,
                               std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
        get_logger(),
        "tb3_spawner ready. Call service: /spawn_tb3 (model default: %s)",
        default_model_.c_str());
}

std::string
Tb3SpawnerNode::resolve_model(const std::string &request_model) const
{
    if (!request_model.empty())
    {
        return request_model;
    }

    const char *env_model = std::getenv("TURTLEBOT3_MODEL");
    if (env_model && std::string(env_model).size() > 0)
    {
        return std::string(env_model);
    }

    return default_model_;
}

std::string Tb3SpawnerNode::resolve_sdf_path(const std::string &model) const
{
    // Prefer turtlebot3_gazebo model SDF because it includes Gazebo plugins.
    const auto share =
        ament_index_cpp::get_package_share_directory("turtlebot3_gazebo");
    const std::string candidate =
        share + "/models/turtlebot3_" + model + "/model.sdf";
    return candidate;
}

std::string Tb3SpawnerNode::read_file(const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        throw std::runtime_error("failed to open file: " + path);
    }

    std::ostringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
}

static std::string patch_sdf_topics(std::string xml, const std::string &name)
{
    const std::string prefix = "/" + name + "/";

    auto replace_all =
        [](std::string &s, const std::string &from, const std::string &to)
    {
        if (from.empty())
            return;
        size_t start_pos = 0;
        while ((start_pos = s.find(from, start_pos)) != std::string::npos)
        {
            s.replace(start_pos, from.length(), to);
            start_pos += to.length();
        }
    };

    // DiffDrive plugin
    replace_all(xml, "<topic>cmd_vel</topic>",
                "<topic>" + prefix + "cmd_vel</topic>");
    replace_all(xml, "<odom_topic>odom</odom_topic>",
                "<odom_topic>" + prefix + "odom</odom_topic>");
    replace_all(xml, "<tf_topic>/tf</tf_topic>",
                "<tf_topic>" + prefix + "tf</tf_topic>");

    // Joint state publisher plugin
    replace_all(xml, "<topic>joint_states</topic>",
                "<topic>" + prefix + "joint_states</topic>");

    // Sensors
    replace_all(xml, "<topic>imu</topic>", "<topic>" + prefix + "imu</topic>");
    replace_all(xml, "<topic>scan</topic>",
                "<topic>" + prefix + "scan</topic>");
    replace_all(xml, "<topic>camera/image_raw</topic>",
                "<topic>" + prefix + "camera/image_raw</topic>");
    replace_all(xml,
                "<camera_info_topic>camera/camera_info</camera_info_topic>",
                "<camera_info_topic>" + prefix +
                    "camera/camera_info</camera_info_topic>");

    return xml;
}

std::string Tb3SpawnerNode::generate_default_name()
{
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    return "tb3_" + std::to_string(now_ns);
}

void Tb3SpawnerNode::handle_spawn(
    const std::shared_ptr<SpawnTb3::Request> request,
    std::shared_ptr<SpawnTb3::Response> response)
{
    const auto model = resolve_model(request->model);
    const auto entity_name =
        request->name.empty() ? generate_default_name() : request->name;

    if (!spawn_client_->wait_for_service(1s))
    {
        response->success = false;
        response->message =
            "Gazebo service not available: " + gazebo_spawn_service_ +
            ". Did you start turtlebot3_gazebo world?";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    std::string sdf_path;
    std::string xml;
    try
    {
        sdf_path = resolve_sdf_path(model);
        xml = read_file(sdf_path);
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message =
            std::string("Failed to load TB3 model XML: ") + e.what();
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    auto req = std::make_shared<SpawnEntity::Request>();
    req->entity_factory.name = entity_name;
    req->entity_factory.allow_renaming = false;
    // Keep default single-robot topic layout when name == model (matches the
    // upstream turtlebot3_gazebo bridge YAML). For a second robot (e.g. tb3_2),
    // prefix Gazebo topics to avoid collisions.
    req->entity_factory.sdf =
        (entity_name == model) ? xml : patch_sdf_topics(xml, entity_name);
    req->entity_factory.sdf_filename = "";
    req->entity_factory.clone_name = "";

    req->entity_factory.pose.position.x = request->x;
    req->entity_factory.pose.position.y = request->y;
    req->entity_factory.pose.position.z = request->z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, request->yaw);
    req->entity_factory.pose.orientation.x = q.x();
    req->entity_factory.pose.orientation.y = q.y();
    req->entity_factory.pose.orientation.z = q.z();
    req->entity_factory.pose.orientation.w = q.w();

    req->entity_factory.relative_to = "world";

    RCLCPP_INFO(
        get_logger(),
        "Spawning TB3 '%s' model='%s' at (%.3f, %.3f, %.3f, yaw=%.3f) using %s",
        entity_name.c_str(), model.c_str(), request->x, request->y, request->z,
        request->yaw, sdf_path.c_str());

    auto future = spawn_client_->async_send_request(req);

    const auto timeout = std::chrono::duration<double>(request_timeout_sec_);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        response->success = false;
        response->message =
            "Timed out waiting for Gazebo create service response";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    const auto result = future.get();
    response->success = result->success;
    response->message = response->success ? "ok" : "failed";

    if (response->success)
    {
        RCLCPP_INFO(get_logger(), "Spawn success: %s",
                    response->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Spawn failed: %s",
                     response->message.c_str());
    }
}

} // namespace ros2_learning_tb3_spawner
