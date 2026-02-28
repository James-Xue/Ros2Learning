cmake_minimum_required(VERSION 3.8)
project(ros2_learning_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # --- 消息 (msg) ---
  "msg/TaskStatus.msg"

  # --- 服务 (srv) ---
  "srv/SpawnTb3.srv"
  "srv/GoTo.srv"

  # --- 动作 (action) ---
  "action/CountTask.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
