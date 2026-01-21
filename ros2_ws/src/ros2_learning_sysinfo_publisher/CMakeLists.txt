cmake_minimum_required(VERSION 3.8)
project(ros2_learning_sysinfo_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 构建可执行文件（多个源文件）
add_executable(sysinfo_publisher
  src/main.cpp
  src/sysinfo_publisher_node.cpp
  src/system_reader.cpp
)

# 添加 include 目录
target_include_directories(sysinfo_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sysinfo_publisher
  rclcpp
  std_msgs
)

# 安装可执行文件
install(TARGETS
  sysinfo_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
