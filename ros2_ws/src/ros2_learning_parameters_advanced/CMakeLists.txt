cmake_minimum_required(VERSION 3.8)
project(ros2_learning_parameters_advanced)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)

# -------- ParamDemoNode 组件 --------
add_library(param_demo_component SHARED src/param_demo_node.cpp)
target_include_directories(param_demo_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(param_demo_component rclcpp rclcpp_components rcl_interfaces)
rclcpp_components_register_node(
  param_demo_component
  PLUGIN "ros2_learning_parameters_advanced::ParamDemoNode"
  EXECUTABLE param_demo
)

# -------- ParamEventListenerNode 组件 --------
add_library(param_event_listener_component SHARED src/param_event_listener_node.cpp)
target_include_directories(param_event_listener_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(param_event_listener_component rclcpp rclcpp_components rcl_interfaces)
rclcpp_components_register_node(
  param_event_listener_component
  PLUGIN "ros2_learning_parameters_advanced::ParamEventListenerNode"
  EXECUTABLE param_event_listener
)

# -------- Install --------
install(TARGETS
  param_demo_component
  param_event_listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
