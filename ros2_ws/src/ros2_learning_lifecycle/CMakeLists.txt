cmake_minimum_required(VERSION 3.8)
project(ros2_learning_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(DEPS rclcpp rclcpp_lifecycle rclcpp_components std_msgs lifecycle_msgs)

# ── 辅助宏：构建并注册一个 rclcpp component 节点库 ─────────────────────
# 用法：add_component(<库名> <源文件> <插件类名> <可执行文件名>)
macro(add_component LIB SRC PLUGIN EXEC)
  add_library(${LIB} SHARED ${SRC})
  target_include_directories(${LIB} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${LIB} ${DEPS})
  rclcpp_components_register_node(${LIB} PLUGIN "${PLUGIN}" EXECUTABLE ${EXEC})
endmacro()

# ── 三个节点组件 ───────────────────────────────────────────────────────
# SensorNode：演示正确的 timer/publisher 生命周期管理
add_component(sensor_node_component
  src/sensor_node.cpp
  "ros2_learning_lifecycle::SensorNode"
  sensor_node)

# ProcessorNode：演示错误注入与 on_error 状态恢复
add_component(processor_node_component
  src/processor_node.cpp
  "ros2_learning_lifecycle::ProcessorNode"
  processor_node)

# LifecycleManagerNode：通过 ChangeState 服务编排多个生命周期节点
add_component(lifecycle_manager_component
  src/lifecycle_manager_node.cpp
  "ros2_learning_lifecycle::LifecycleManagerNode"
  lifecycle_manager_node)

# ── 安装 ───────────────────────────────────────────────────────────────
install(TARGETS
  sensor_node_component processor_node_component lifecycle_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ── 测试 ───────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # 跳过与项目风格不兼容的静态检查（4-space C++、中文注释、双引号 launch）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lifecycle_transitions
    test/test_lifecycle_transitions.cpp
    src/sensor_node.cpp src/processor_node.cpp)
  target_include_directories(test_lifecycle_transitions PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_lifecycle_transitions ${DEPS})
endif()

ament_package()
