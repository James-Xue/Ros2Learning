cmake_minimum_required(VERSION 3.8)
project(ros2_learning_tf_quaternion_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)

# 可执行文件 1: 四元数演示
add_executable(quaternion_demo 
  src/quaternion_demo.cpp
  src/quaternion_demo_main.cpp
)
ament_target_dependencies(quaternion_demo
  rclcpp
  tf2
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

# 添加include目录
target_include_directories(quaternion_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 可执行文件 2: TF 广播器演示
add_executable(tf_broadcaster_demo 
  src/tf_broadcaster_demo.cpp
  src/tf_broadcaster_demo_main.cpp
)
ament_target_dependencies(tf_broadcaster_demo
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

target_include_directories(tf_broadcaster_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 可执行文件 3: TF 监听器演示
add_executable(tf_listener_demo 
  src/tf_listener_demo.cpp
  src/tf_listener_demo_main.cpp
)
ament_target_dependencies(tf_listener_demo
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

target_include_directories(tf_listener_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS
  quaternion_demo
  tf_broadcaster_demo
  tf_listener_demo
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 RViz 配置
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装文档
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
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
