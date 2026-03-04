cmake_minimum_required(VERSION 3.8)
project(ros2_learning_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_learning_custom_interfaces REQUIRED)

set(DEPS rclcpp rclcpp_action rclcpp_components ros2_learning_custom_interfaces)

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

# ── Action Server 组件 ─────────────────────────────────────────────────
add_component(count_action_server_component
  src/count_action_server.cpp
  "ros2_learning_actions::CountActionServer"
  count_action_server)

# ── Action Client 组件 ─────────────────────────────────────────────────
add_component(count_action_client_component
  src/count_action_client.cpp
  "ros2_learning_actions::CountActionClient"
  count_action_client)

# ── 安装 ───────────────────────────────────────────────────────────────
install(TARGETS
  count_action_server_component count_action_client_component
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

  ament_add_gtest(test_count_action
    test/test_count_action.cpp
    src/count_action_server.cpp)
  target_include_directories(test_count_action PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_count_action ${DEPS})
endif()

ament_package()
