cmake_minimum_required(VERSION 3.8)
project(active_marker)

# Find ament_cmake first
find_package(ament_cmake REQUIRED)

# Find ROS2 packages
set(ros2_required_packages
  rclcpp
  std_msgs
)
foreach(ros2_required_package IN LISTS ros2_required_packages)
  find_package(${ros2_required_package} REQUIRED)
endforeach()

# Set compile files
file(GLOB_RECURSE src_files 
  LIST_DIRECTORIES false 
  "src/active_marker/*")
file(GLOB app_files 
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/src
  "src/*")

# Set library
add_library(active_marker
  ${src_files}
)
target_include_directories(active_marker 
  PRIVATE include
  INTERFACE include
)

ament_target_dependencies(active_marker
  rclcpp
  std_msgs
)

# Initialize executors variable
set(executors "")

# Make executors
foreach(app_file IN LISTS app_files)
  string(FIND ${app_file} . index)
  string(SUBSTRING ${app_file} 0 ${index} executor)
  list(APPEND executors ${executor})
  add_executable(${executor}
    src/${app_file}
  )
  target_link_libraries(${executor} active_marker)
endforeach()

# Install executors
install(TARGETS
  ${executors}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
