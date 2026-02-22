cmake_minimum_required(VERSION 3.8)
project(ros2_learning_qos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Executable 1: QoS Producer (Simulates Rover)
add_executable(qos_producer src/qos_producer.cpp)
ament_target_dependencies(qos_producer rclcpp std_msgs sensor_msgs)

# Executable 2: QoS Consumer (Simulates Ground Control)
add_executable(qos_consumer src/qos_consumer.cpp)
ament_target_dependencies(qos_consumer rclcpp std_msgs sensor_msgs)

install(TARGETS
  qos_producer
  qos_consumer
  DESTINATION lib/${PROJECT_NAME}
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
