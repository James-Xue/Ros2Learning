cmake_minimum_required(VERSION 3.8)
project(ros2_learning_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ============================================================
# SensorNode — LifecycleNode，演示正确的 timer/publisher 管理
# ============================================================
add_library(sensor_node_component SHARED src/sensor_node.cpp)
target_include_directories(sensor_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sensor_node_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs lifecycle_msgs)
rclcpp_components_register_node(sensor_node_component
  PLUGIN "ros2_learning_lifecycle::SensorNode"
  EXECUTABLE sensor_node)

# ============================================================
# ProcessorNode — LifecycleNode，演示错误注入与 on_error 恢复
# ============================================================
add_library(processor_node_component SHARED src/processor_node.cpp)
target_include_directories(processor_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(processor_node_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs lifecycle_msgs)
rclcpp_components_register_node(processor_node_component
  PLUGIN "ros2_learning_lifecycle::ProcessorNode"
  EXECUTABLE processor_node)

# ============================================================
# LifecycleManagerNode — 普通 Node，通过 ChangeState 服务编排多节点
# ============================================================
add_library(lifecycle_manager_component SHARED src/lifecycle_manager_node.cpp)
target_include_directories(lifecycle_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lifecycle_manager_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs lifecycle_msgs)
rclcpp_components_register_node(lifecycle_manager_component
  PLUGIN "ros2_learning_lifecycle::LifecycleManagerNode"
  EXECUTABLE lifecycle_manager_node)

# ============================================================
# 安装
# ============================================================
install(TARGETS
  sensor_node_component
  processor_node_component
  lifecycle_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ============================================================
# 测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)  # 项目使用 4-space C++ 风格，与 uncrustify 默认不符
  set(ament_cmake_flake8_FOUND TRUE)      # launch 文件使用双引号和中文注释，不符合 flake8 插件要求
  set(ament_cmake_pep257_FOUND TRUE)      # docstring 为中文说明，不符合 pep257 英文格式要求
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lifecycle_transitions
    test/test_lifecycle_transitions.cpp
    src/sensor_node.cpp
    src/processor_node.cpp)
  target_include_directories(test_lifecycle_transitions PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_lifecycle_transitions
    rclcpp rclcpp_lifecycle rclcpp_components std_msgs lifecycle_msgs)
endif()

ament_package()
