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
 * @file tb3_spawner_node.hpp
 * @brief 在 Gazebo 中生成 TurtleBot3 模型的节点
 *
 * 该节点提供了一个服务，用于在 Gazebo 中生成 TurtleBot3 模型。
 * 它支持自动的话题重映射（Remapping），以支持多机器人场景。
 */

#ifndef ROS2_LEARNING_TB3_SPAWNER__TB3_SPAWNER_NODE_HPP_
#define ROS2_LEARNING_TB3_SPAWNER__TB3_SPAWNER_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "ros2_learning_tb3_spawner/srv/spawn_tb3.hpp"

namespace ros2_learning_tb3_spawner
{

/**
 * @class Tb3SpawnerNode
 * @brief 管理 TurtleBot3 实体在 Gazebo 中的生成。
 *
 * 该节点提供了一个 `spawn_tb3` 服务，它是对标准 `ros_gz_interfaces` 生成服务的封装。
 * 它会自动查找正确的模型路径，并可选择修补 SDF 文件以支持多机器人的话题重映射。
 */
class Tb3SpawnerNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   *
   * 初始化参数，并设置生成服务的服务端和客户端。
   */
  Tb3SpawnerNode();

private:
  using SpawnTb3 = ros2_learning_tb3_spawner::srv::SpawnTb3;
  using SpawnEntity = ros_gz_interfaces::srv::SpawnEntity;

  /**
   * @brief 处理生成服务请求的回调函数
   *
   * @param request 包含模型类型、位置和名称的生成请求。
   * @param response 指示成功或失败的响应。
   */
  void handle_spawn(
    const std::shared_ptr<SpawnTb3::Request> request,
    std::shared_ptr<SpawnTb3::Response> response);

  /**
   * @brief 解析 TurtleBot3 模型名称
   *
   * 如果请求中指定了模型，则使用该模型。否则，检查环境变量 `TURTLEBOT3_MODEL`，
   * 如果环境变量也未设置，则回退到默认参数。
   *
   * @param request_model 请求的模型字符串。
   * @return std::string 解析后的模型名称（例如 "burger", "waffle", "waffle_pi"）。
   */
  std::string resolve_model(const std::string & request_model) const;

  /**
   * @brief 定位指定模型的 SDF 文件路径
   *
   * @param model 模型名称。
   * @return std::string model.sdf 文件的绝对路径。
   */
  std::string resolve_sdf_path(const std::string & model) const;

  /**
   * @brief 读取文件内容到字符串中
   *
   * @param path 文件路径。
   * @return std::string 文件内容。
   */
  static std::string read_file(const std::string & path);

  /**
   * @brief 生成一个唯一的默认机器人名称
   *
   * @return std::string 基于当前时间戳的名称，例如 "tb3_123456789"。
   */
  static std::string generate_default_name();

  // --- 成员变量 ---

  /// 用于与 Gazebo 生成服务通信的客户端
  rclcpp::Client<SpawnEntity>::SharedPtr spawn_client_;

  /// 本节点提供的服务
  rclcpp::Service<SpawnTb3>::SharedPtr spawn_service_;

  /// 默认机器人模型（来自参数或环境变量）
  std::string default_model_;

  /// 要调用的 Gazebo 生成服务名称
  std::string gazebo_spawn_service_;

  /// Gazebo 服务请求的超时时间
  double request_timeout_sec_;
};

}  // namespace ros2_learning_tb3_spawner

#endif  // ROS2_LEARNING_TB3_SPAWNER__TB3_SPAWNER_NODE_HPP_
