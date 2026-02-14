cmake_minimum_required(VERSION 3.8)
project(ros2_learning_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# ----------------- Talker 组件 -----------------
add_library(talker_component SHARED src/talker.cpp)
target_include_directories(talker_component PRIVATE include)
ament_target_dependencies(talker_component rclcpp rclcpp_components rclcpp_lifecycle std_msgs)
# 注册 Talker 组件（生成插件信息，可选生成可执行文件 `talker`）
rclcpp_components_register_node(talker_component PLUGIN "ros2_learning_cpp::TalkerNode" EXECUTABLE talker)

# ----------------- Listener 组件 -----------------
add_library(listener_component SHARED src/listener.cpp)
target_include_directories(listener_component PRIVATE include)
ament_target_dependencies(listener_component rclcpp rclcpp_components std_msgs)
# 注册 Listener 组件（生成插件信息，可选生成可执行文件 `listener`）
rclcpp_components_register_node(listener_component PLUGIN "ros2_learning_cpp::ListenerNode" EXECUTABLE listener)

# ----------------- Install -----------------
install(TARGETS
  talker_component
  listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(talker_component listener_component)
ament_export_dependencies(rclcpp_components)

ament_package()
