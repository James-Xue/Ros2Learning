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
 * @file tb3_spawner_main.cpp
 * @brief tb3_spawner 节点的入口函数
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ros2_learning_tb3_spawner/tb3_spawner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ros2_learning_tb3_spawner::Tb3SpawnerNode>();

  // 由于我们在服务回调中可能需要等待 Gazebo 的响应（这是一个阻塞操作），
  // 也就是在一个 Service Callback (spawn_tb3) 中去调用另一个 Service Client (spawn_entity)。
  // 如果使用默认的 SingleThreadedExecutor，在等待 Client 响应时，整个节点会卡住，
  // 导致无法处理 Client 返回的 Response，从而造成死锁。
  //
  // 因此，必须使用 MultiThreadedExecutor，
  // 让服务回调在一个线程中运行，而让 Client 的响应处理在另一个线程中运行。
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
