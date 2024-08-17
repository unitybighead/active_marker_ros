cmake_minimum_required(VERSION 3.8)
project(active_marker)

# Find ament_cmake_auto first
find_package(ament_cmake_auto REQUIRED)

# Use ament_cmake_auto to set up the package
ament_auto_find_build_dependencies()

# Add active_marker_lib
file(GLOB LIB_SOURCES "active_marker_lib/src/*.cpp")
ament_auto_add_library(active_marker_lib STATIC ${LIB_SOURCES})

target_compile_options(active_marker_lib PRIVATE -fPIC)

# Add main library
file(GLOB MAIN_LIB_SOURCES "src/active_marker/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${MAIN_LIB_SOURCES})

# Find executables automatically
file(GLOB EXECUTABLE_SOURCES "src/*.cpp")

foreach(EXECUTABLE_SOURCE ${EXECUTABLE_SOURCES})
  # Get the filename without the extension (e.g., color_pub)
  get_filename_component(EXECUTABLE_NAME ${EXECUTABLE_SOURCE} NAME_WE)
  
  ament_auto_add_executable(${EXECUTABLE_NAME} ${EXECUTABLE_SOURCE})
endforeach()

# Install executables and libraries
ament_auto_package()
