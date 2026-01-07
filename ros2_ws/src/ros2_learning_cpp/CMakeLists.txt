cmake_minimum_required(VERSION 3.8)
project(ros2_learning_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker
  src/talker.cpp
  src/talker_main.cpp
)
target_include_directories(talker PRIVATE include)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener
  src/listener.cpp
  src/listener_main.cpp
)
target_include_directories(listener PRIVATE include)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
