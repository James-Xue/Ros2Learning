cmake_minimum_required(VERSION 3.8)
project(ros2_learning_qos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# ============================================================
# 组件化架构 (Composition)
# 每个节点编译为共享库(.so)，可被 component_container 动态加载
# rclcpp_components_register_nodes 同时会自动生成独立可执行文件
# ============================================================

# Component 1: RoverNode (QoS Producer)
add_library(rover_node SHARED src/rover_node.cpp)
ament_target_dependencies(rover_node rclcpp rclcpp_components std_msgs sensor_msgs)
rclcpp_components_register_node(rover_node
  PLUGIN "ros2_learning_qos::RoverNode"
  EXECUTABLE qos_producer
)

# Component 2: MissionControlNode (QoS Consumer)
add_library(mission_control_node SHARED src/mission_control_node.cpp)
ament_target_dependencies(mission_control_node rclcpp rclcpp_components std_msgs sensor_msgs)
rclcpp_components_register_node(mission_control_node
  PLUGIN "ros2_learning_qos::MissionControlNode"
  EXECUTABLE qos_consumer
)

install(TARGETS
  rover_node
  mission_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
