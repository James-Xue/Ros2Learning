cmake_minimum_required(VERSION 3.8)
project(ros2_learning_sysinfo_qt_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Prefer Qt6 if available, fallback to Qt5.
find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
  set(QT_WIDGETS_LIB Qt6::Widgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_WIDGETS_LIB Qt5::Widgets)
endif()

# 构建可执行文件（多个源文件）
add_executable(sysinfo_qt_viewer
  src/main.cpp
  src/sysinfo_qt_node.cpp
  src/json_formatter.cpp
)

# 添加 include 目录
target_include_directories(sysinfo_qt_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(sysinfo_qt_viewer
  ${QT_WIDGETS_LIB}
)

ament_target_dependencies(sysinfo_qt_viewer
  rclcpp
  std_msgs
)

# 安装可执行文件
install(TARGETS
  sysinfo_qt_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
