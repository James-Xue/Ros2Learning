cmake_minimum_required(VERSION 3.8)
project(ros2_learning_arm_basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 添加include目录
include_directories(include)

# 添加可执行文件：arm_position_controller
# 现在使用多个源文件
add_executable(arm_position_controller 
  src/arm_position_controller.cpp
  src/arm_position_controller_main.cpp
)

ament_target_dependencies(arm_position_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)

# 安装可执行文件
install(TARGETS
  arm_position_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
