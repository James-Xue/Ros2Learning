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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnTb3.srv"
)

add_executable(tb3_spawner
  src/tb3_spawner_node.cpp
  src/tb3_spawner_main.cpp
)

ament_target_dependencies(tb3_spawner
  rclcpp
  ros_gz_interfaces
  geometry_msgs
  tf2
  ament_index_cpp
)

# 使用新的 rosidl typesupport API（替代已弃用的 rosidl_target_interfaces）
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tb3_spawner "${cpp_typesupport_target}")

install(TARGETS
  tb3_spawner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
