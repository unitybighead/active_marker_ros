cmake_minimum_required(VERSION 3.8)
project(active_marker)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
 src/color_pub.cpp 
 include/active_marker/color_pub.hpp
 src/color_sub.cpp
 include/active_marker/color_sub.hpp
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()