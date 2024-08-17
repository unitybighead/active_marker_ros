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
file(GLOB LIB_SOURCES "active_marker_lib/src/*.cpp")
ament_auto_add_library(active_marker_lib STATIC ${LIB_SOURCES})

target_include_directories(active_marker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/active_marker_lib/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(active_marker_lib PRIVATE -fPIC)

# Add main library
file(GLOB MAIN_LIB_SOURCES "src/active_marker/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${MAIN_LIB_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Find executables automatically
file(GLOB EXECUTABLE_SOURCES "src/*.cpp")

foreach(EXECUTABLE_SOURCE ${EXECUTABLE_SOURCES})
  # Get the filename without the extension (e.g., color_pub)
  get_filename_component(EXECUTABLE_NAME ${EXECUTABLE_SOURCE} NAME_WE)
  
  ament_auto_add_executable(${EXECUTABLE_NAME} ${EXECUTABLE_SOURCE})
  target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME})
endforeach()

# Link active_marker_lib to specific executables
target_link_libraries(illuminance_pub active_marker_lib)
target_link_libraries(illuminance_sub active_marker_lib)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install executables and libraries
ament_auto_package()
