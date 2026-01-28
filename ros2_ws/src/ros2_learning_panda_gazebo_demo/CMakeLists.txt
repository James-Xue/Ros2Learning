cmake_minimum_required(VERSION 3.8)
project(ros2_learning_panda_gazebo_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(panda_joint_commander
  src/panda_joint_commander.cpp
  src/panda_joint_commander_main.cpp
)

ament_target_dependencies(panda_joint_commander
  rclcpp
  std_msgs
)

target_include_directories(panda_joint_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  panda_joint_commander
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
