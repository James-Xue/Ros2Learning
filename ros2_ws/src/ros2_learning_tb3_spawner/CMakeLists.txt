cmake_minimum_required(VERSION 3.8)
project(ros2_learning_tb3_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros2_learning_custom_interfaces REQUIRED)

add_executable(tb3_spawner
  src/tb3_spawner_node.cpp
  src/tb3_spawner_main.cpp
)

target_include_directories(tb3_spawner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tb3_spawner
  rclcpp
  ros_gz_interfaces
  geometry_msgs
  tf2
  ament_index_cpp
  ros2_learning_custom_interfaces
)

install(TARGETS
  tb3_spawner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ros2_learning_custom_interfaces)

ament_package()
