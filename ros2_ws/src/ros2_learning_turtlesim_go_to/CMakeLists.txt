cmake_minimum_required(VERSION 3.8)
project(ros2_learning_turtlesim_go_to)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoTo.srv"
)

add_executable(turtlesim_go_to_server
  src/turtlesim_go_to_server_node.cpp
  src/turtlesim_go_to_server_main.cpp
)

target_include_directories(turtlesim_go_to_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(turtlesim_go_to_server
  rclcpp
  geometry_msgs
  turtlesim
)

# Link generated service typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(turtlesim_go_to_server "${cpp_typesupport_target}")

add_executable(turtlesim_go_to_client
  src/turtlesim_go_to_client_node.cpp
  src/turtlesim_go_to_client_main.cpp
)

ament_target_dependencies(turtlesim_go_to_client
  rclcpp
)

# Client also needs the generated service symbols
target_link_libraries(turtlesim_go_to_client "${cpp_typesupport_target}")

target_include_directories(turtlesim_go_to_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  turtlesim_go_to_server
  turtlesim_go_to_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
