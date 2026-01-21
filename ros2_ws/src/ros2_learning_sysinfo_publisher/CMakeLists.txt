cmake_minimum_required(VERSION 3.8)
project(ros2_learning_sysinfo_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sysinfo_publisher
  src/sysinfo_publisher.cpp
)

ament_target_dependencies(sysinfo_publisher
  rclcpp
  std_msgs
)

install(TARGETS
  sysinfo_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
