cmake_minimum_required(VERSION 3.8)
project(ros2_learning_control_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 依赖查找
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ============================================================
# Hardware Interface 插件库
# ============================================================
add_library(simple_joint_hardware SHARED
  src/simple_joint_hardware.cpp
)

target_include_directories(simple_joint_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simple_joint_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 导出插件描述文件
pluginlib_export_plugin_description_file(hardware_interface simple_joint_hardware_plugin.xml)

# ============================================================
# 安装
# ============================================================
install(TARGETS simple_joint_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# ament 导出
# ============================================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
