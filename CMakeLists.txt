cmake_minimum_required(VERSION 3.14)
project(dv_processing_driver LANGUAGES CXX)

# Compiler settings
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)

  # ignore the private-field-unused warning with clang
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-unused-private-field)
  endif()
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add dv-processing as submodule
add_subdirectory(thirdparty/dv-processing EXCLUDE_FROM_ALL)

# ROS2 + dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dvs_msgs REQUIRED)  # EventArray message
find_package(OpenCV REQUIRED)
find_package(ZLIB REQUIRED)

# Executable definition
add_executable(dv_processing_driver_node
  src/driver_node.cpp
)

# Include directories
target_include_directories(dv_processing_driver_node
  SYSTEM
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

# Link libraries (dv-processing must be here)
target_link_libraries(dv_processing_driver_node
  dv::processing
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}
)

# Ament dependencies (proper ROS include dirs & linking)
ament_target_dependencies(dv_processing_driver_node
  rclcpp
  sensor_msgs
  dvs_msgs
)

# Install binaries
install(TARGETS dv_processing_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
