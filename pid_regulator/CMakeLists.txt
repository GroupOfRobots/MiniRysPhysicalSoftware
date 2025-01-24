cmake_minimum_required(VERSION 3.8)
project(pid_regulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(pid_regulator SHARED DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

ament_auto_package()
