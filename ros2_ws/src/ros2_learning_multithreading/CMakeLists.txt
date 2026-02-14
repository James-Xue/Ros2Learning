cmake_minimum_required(VERSION 3.8)
project(ros2_learning_multithreading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(executor_demo
  src/executor_demo_main.cpp
  src/blocking_node.cpp
)
ament_target_dependencies(executor_demo rclcpp)

install(TARGETS
  executor_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
