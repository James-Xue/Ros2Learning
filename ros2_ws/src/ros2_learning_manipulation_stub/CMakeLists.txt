cmake_minimum_required(VERSION 3.8)
project(ros2_learning_manipulation_stub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(manipulation_stub
  src/manipulation_stub.cpp
  src/manipulation_stub_main.cpp
)

ament_target_dependencies(manipulation_stub
  rclcpp
  std_srvs
)

target_include_directories(manipulation_stub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  manipulation_stub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
