cmake_minimum_required(VERSION 3.8)
project(ros2_learning_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# 1. Math Library (纯数学库，无 ROS 依赖)
add_library(kinematics_lib
  src/planar_2dof.cpp
)
target_include_directories(kinematics_lib PRIVATE include)

# 2. Demo Node (ROS 节点)
add_executable(ik_demo_node
  src/ik_demo_node.cpp
)
target_link_libraries(ik_demo_node kinematics_lib)
ament_target_dependencies(ik_demo_node rclcpp geometry_msgs)

install(TARGETS
  kinematics_lib
  ik_demo_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
