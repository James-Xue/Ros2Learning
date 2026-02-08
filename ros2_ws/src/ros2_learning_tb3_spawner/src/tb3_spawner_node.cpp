// Copyright 2026 user
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file tb3_spawner_node.cpp
 * @brief Tb3SpawnerNode 类的实现
 */

#include "ros2_learning_tb3_spawner/tb3_spawner_node.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace ros2_learning_tb3_spawner
{

using namespace std::chrono_literals;

Tb3SpawnerNode::Tb3SpawnerNode()
: Node("tb3_spawner")
{
  // 默认参数适用于常见的教程流程:
  // - 用户通过 turtlebot3_gazebo 启动 Gazebo
  // - 然后启动此生成节点
  gazebo_spawn_service_ = this->declare_parameter<std::string>(
    "gazebo_spawn_service", "/spawn_entity");

  // 尝试从环境变量获取默认模型
  const char * env_model = std::getenv("TURTLEBOT3_MODEL");
  default_model_ = this->declare_parameter<std::string>(
    "default_model", env_model ? std::string(env_model) : "waffle_pi");

  request_timeout_sec_ =
    this->declare_parameter<double>("request_timeout_sec", 10.0);

  // 使用标准的 ros_gz_interfaces 服务客户端
  spawn_client_ = this->create_client<SpawnEntity>(gazebo_spawn_service_);

  // 服务回调可能会阻塞等待 Gazebo 的响应。
  // main 函数中的 MultiThreadedExecutor 会确保客户端响应能够被及时处理。
  spawn_service_ = this->create_service<SpawnTb3>(
    "spawn_tb3", std::bind(
      &Tb3SpawnerNode::handle_spawn, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "tb3_spawner 就绪. 服务可用地址: /spawn_tb3 (默认模型: %s)",
    default_model_.c_str());
}

std::string
Tb3SpawnerNode::resolve_model(const std::string & request_model) const
{
  // 如果请求中指定了模型名称，直接使用
  if (!request_model.empty()) {
    return request_model;
  }

  // 否则尝试使用环境变量
  const char * env_model = std::getenv("TURTLEBOT3_MODEL");
  if (env_model && std::string(env_model).size() > 0) {
    return std::string(env_model);
  }

  // 最后回退到默认参数
  return default_model_;
}

std::string Tb3SpawnerNode::resolve_sdf_path(const std::string & model) const
{
  // 优先使用 turtlebot3_gazebo 中的 SDF 模型，因为它包含了 Gazebo 插件。
  std::string package_share_directory;
  try {
    package_share_directory = ament_index_cpp::get_package_share_directory("turtlebot3_gazebo");
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("无法找到包 'turtlebot3_gazebo': ") + e.what());
  }

  // 构造 SDF 文件的完整路径
  std::filesystem::path sdf_path =
    std::filesystem::path(package_share_directory) / "models" / ("turtlebot3_" + model) / "model.sdf";

  if (!std::filesystem::exists(sdf_path)) {
    throw std::runtime_error("模型 SDF 文件未找到: " + sdf_path.string());
  }

  return sdf_path.string();
}

std::string Tb3SpawnerNode::read_file(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("无法打开文件: " + path);
  }

  std::ostringstream ss;
  ss << ifs.rdbuf();
  return ss.str();
}

/**
 * @brief 修补 SDF XML 内容以实现话题重映射
 *
 * 为了支持多机器人仿真，我们需要避免话题冲突。
 * 此函数会在关键话题前加上机器人名称作为前缀。
 *
 * @param xml 原始 SDF XML 内容
 * @param name 机器人名称（命名空间前缀）
 * @return std::string 修改后的 XML 内容
 */
static std::string patch_sdf_topics(std::string xml, const std::string & name)
{
  const std::string prefix = "/" + name + "/";

  // 一个简单的辅助 Lambda，用于替换字符串中的所有匹配项
  auto replace_all =
    [](std::string & s, const std::string & from, const std::string & to)
    {
      if (from.empty()) {
        return;
      }
      size_t start_pos = 0;
      while ((start_pos = s.find(from, start_pos)) != std::string::npos) {
        s.replace(start_pos, from.length(), to);
        start_pos += to.length();
      }
    };

  // DiffDrive 插件话题
  replace_all(
    xml, "<topic>cmd_vel</topic>",
    "<topic>" + prefix + "cmd_vel</topic>");
  replace_all(
    xml, "<odom_topic>odom</odom_topic>",
    "<odom_topic>" + prefix + "odom</odom_topic>");
  replace_all(
    xml, "<tf_topic>/tf</tf_topic>",
    "<tf_topic>" + prefix + "tf</tf_topic>");

  // Joint State Publisher 插件话题
  replace_all(
    xml, "<topic>joint_states</topic>",
    "<topic>" + prefix + "joint_states</topic>");

  // 传感器话题
  replace_all(xml, "<topic>imu</topic>", "<topic>" + prefix + "imu</topic>");
  replace_all(
    xml, "<topic>scan</topic>",
    "<topic>" + prefix + "scan</topic>");
  replace_all(
    xml, "<topic>camera/image_raw</topic>",
    "<topic>" + prefix + "camera/image_raw</topic>");
  replace_all(
    xml,
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

  RCLCPP_INFO(get_logger(), "收到生成请求: 机器人 '%s' (模型: %s)", entity_name.c_str(), model.c_str());

  // 等待 Gazebo 服务就绪
  if (!spawn_client_->wait_for_service(std::chrono::seconds(2))) {
    response->success = false;
    response->message =
      "Gazebo 生成服务 (" + gazebo_spawn_service_ + ") 不可用。 "
      "您是否已经启动了 Gazebo 世界 (例如: ros2 launch turtlebot3_gazebo ...)?";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  std::string sdf_path;
  std::string xml;
  try {
    sdf_path = resolve_sdf_path(model);
    xml = read_file(sdf_path);
  } catch (const std::exception & e) {
    response->success = false;
    response->message =
      std::string("加载 TurtleBot3 模型 XML 失败: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto req = std::make_shared<SpawnEntity::Request>();
  req->entity_factory.name = entity_name;
  req->entity_factory.allow_renaming = false;
  
  // 如果实体名称与模型名称相同（默认情况），保持 upstream 的话题布局。
  // 如果是第二个机器人（例如 tb3_2），则加上前缀以避免话题冲突。
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
    "向 Gazebo 发送生成请求: 实体 '%s' 位于 (%.2f, %.2f, %.2f)",
    entity_name.c_str(), request->x, request->y, request->z);

  auto future = spawn_client_->async_send_request(req);

  const auto timeout = std::chrono::duration<double>(request_timeout_sec_);
  if (future.wait_for(timeout) != std::future_status::ready) {
    response->success = false;
    response->message =
      "等待 Gazebo 生成服务响应超时 (" + std::to_string(request_timeout_sec_) + "s)";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const auto result = future.get();
  response->success = result->success;
  // 如果 Gazebo 返回失败但没有消息，我们使用默认消息
  response->message = response->success ? "生成成功" : "生成失败 (Gazebo 拒绝请求)";

  if (response->success) {
    RCLCPP_INFO(
      get_logger(), "生成成功: %s",
      response->message.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(), "生成失败: %s",
      response->message.c_str());
  }
}

}  // namespace ros2_learning_tb3_spawner
