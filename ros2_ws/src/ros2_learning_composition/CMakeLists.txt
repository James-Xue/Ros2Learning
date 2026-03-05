cmake_minimum_required(VERSION 3.8)
project(ros2_learning_composition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(DEPS rclcpp rclcpp_components rclcpp_lifecycle sensor_msgs rcl_interfaces)

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

# ── 三个节点组件 ───────────────────────────────────────────────────────
# ImageProducer：定时生成模拟图像，使用 unique_ptr 发布实现零拷贝
add_component(image_producer_component
  src/image_producer.cpp
  "ros2_learning_composition::ImageProducer"
  image_producer)

# ImageProcessor：订阅原始图像，反转像素后转发，验证零拷贝地址
add_component(image_processor_component
  src/image_processor.cpp
  "ros2_learning_composition::ImageProcessor"
  image_processor)

# ImageSaver：LifecycleNode 组件，统计帧数与延迟
add_component(image_saver_component
  src/image_saver.cpp
  "ros2_learning_composition::ImageSaver"
  image_saver)

# ── 安装 ───────────────────────────────────────────────────────────────
install(TARGETS
  image_producer_component image_processor_component image_saver_component
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

  find_package(lifecycle_msgs REQUIRED)

  ament_add_gtest(test_composition test/test_composition.cpp)
  target_include_directories(test_composition PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_composition
    image_producer_component image_processor_component image_saver_component)
  ament_target_dependencies(test_composition ${DEPS} lifecycle_msgs)
endif()

ament_package()
