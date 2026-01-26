#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros_gz_interfaces/srv/spawn_entity.hpp"

#include "ros2_learning_tb3_spawner/srv/spawn_tb3.hpp"

namespace ros2_learning_tb3_spawner
{

class Tb3SpawnerNode : public rclcpp::Node
{
  public:
    Tb3SpawnerNode();

  private:
    using SpawnTb3 = ros2_learning_tb3_spawner::srv::SpawnTb3;
    using SpawnEntity = ros_gz_interfaces::srv::SpawnEntity;

    void handle_spawn(const std::shared_ptr<SpawnTb3::Request> request,
                      std::shared_ptr<SpawnTb3::Response> response);

    std::string resolve_model(const std::string &request_model) const;
    std::string resolve_sdf_path(const std::string &model) const;
    static std::string read_file(const std::string &path);
    static std::string generate_default_name();

    rclcpp::Client<SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Service<SpawnTb3>::SharedPtr spawn_service_;

    std::string default_model_;
    std::string gazebo_spawn_service_;
    double request_timeout_sec_;
};

} // namespace ros2_learning_tb3_spawner
