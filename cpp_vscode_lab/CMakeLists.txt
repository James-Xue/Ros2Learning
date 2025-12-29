cmake_minimum_required(VERSION 3.16)
project(cpp_vscode_lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_library(project_warnings INTERFACE)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(project_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

function(add_demo demo_name demo_source)
  add_executable(${demo_name} ${demo_source})
  target_link_libraries(${demo_name} PRIVATE project_warnings)
endfunction()

add_demo(01_hello src/01_hello.cpp)
add_demo(02_args src/02_args.cpp)
add_demo(03_vector_sort src/03_vector_sort.cpp)
add_demo(04_class_raii src/04_class_raii.cpp)
add_demo(05_threads src/05_threads.cpp)
