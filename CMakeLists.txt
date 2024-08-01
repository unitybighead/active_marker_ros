cmake_minimum_required(VERSION 3.8)
project(active_marker)

# Find ament_cmake_auto first
find_package(ament_cmake_auto REQUIRED)

# Find ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Use ament_cmake_auto to set up the package
ament_auto_find_build_dependencies()

# Add active_marker_lib
ament_auto_add_library(active_marker_lib STATIC
  active_marker_lib/src/udp.cpp
)

target_include_directories(active_marker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/active_marker_lib/include>
  $<INSTALL_INTERFACE:include>
)

# Add main library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/active_marker/color_pub.cpp
  src/active_marker/color_sub.cpp
  src/active_marker/illuminance_pub.cpp
  src/active_marker/illuminance_sub.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add executables
ament_auto_add_executable(color_pub src/color_pub.cpp)
ament_auto_add_executable(color_sub src/color_sub.cpp)
ament_auto_add_executable(illuminance_pub src/illuminance_pub.cpp)
ament_auto_add_executable(illuminance_sub src/illuminance_sub.cpp)

# Link libraries
target_link_libraries(color_pub ${PROJECT_NAME} active_marker_lib)
target_link_libraries(color_sub ${PROJECT_NAME} active_marker_lib)
target_link_libraries(illuminance_pub ${PROJECT_NAME} active_marker_lib)
target_link_libraries(illuminance_sub ${PROJECT_NAME} active_marker_lib)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install executables and libraries
ament_auto_package()
