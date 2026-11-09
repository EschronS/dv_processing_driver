cmake_minimum_required(VERSION 3.22)
project(dv_processing_driver LANGUAGES CXX)

# Default to an optimized build. Without this, a plain `colcon build` leaves
# CMAKE_BUILD_TYPE empty, which disables -O2/-O3 and NDEBUG (dv-processing then
# keeps its per-event runtime bounds checks enabled).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

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
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Apply local patches to the vendored dv-processing submodule.
set(DV_PROCESSING_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/dv-processing")
set(DV_PROCESSING_EVENT_RINGBUFFER_PATCH
  "${CMAKE_CURRENT_SOURCE_DIR}/patches/dv-processing-event-ringbuffer.patch")

if(NOT EXISTS "${DV_PROCESSING_SOURCE_DIR}/include/dv-processing/io/camera/camera_input_base.hpp")
  message(FATAL_ERROR
    "dv-processing submodule is missing. Run: git submodule update --init --recursive")
endif()

find_package(Git QUIET)
if(NOT Git_FOUND)
  message(FATAL_ERROR "git is required to apply the vendored dv-processing patch")
endif()

execute_process(
  COMMAND "${GIT_EXECUTABLE}" -C "${DV_PROCESSING_SOURCE_DIR}" apply --check
          "${DV_PROCESSING_EVENT_RINGBUFFER_PATCH}"
  RESULT_VARIABLE DV_PATCH_CHECK_RESULT
  OUTPUT_QUIET
  ERROR_QUIET)

if(DV_PATCH_CHECK_RESULT EQUAL 0)
  message(STATUS "Applying dv-processing event ring-buffer patch")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${DV_PROCESSING_SOURCE_DIR}" apply
            "${DV_PROCESSING_EVENT_RINGBUFFER_PATCH}"
    RESULT_VARIABLE DV_PATCH_APPLY_RESULT
    ERROR_VARIABLE DV_PATCH_APPLY_ERROR)
  if(NOT DV_PATCH_APPLY_RESULT EQUAL 0)
    message(FATAL_ERROR
      "Failed to apply ${DV_PROCESSING_EVENT_RINGBUFFER_PATCH}: ${DV_PATCH_APPLY_ERROR}")
  endif()
else()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${DV_PROCESSING_SOURCE_DIR}" apply --reverse --check
            "${DV_PROCESSING_EVENT_RINGBUFFER_PATCH}"
    RESULT_VARIABLE DV_PATCH_REVERSE_CHECK_RESULT
    OUTPUT_QUIET
    ERROR_QUIET)
  if(DV_PATCH_REVERSE_CHECK_RESULT EQUAL 0)
    message(STATUS "dv-processing event ring-buffer patch already applied")
  else()
    message(FATAL_ERROR
      "Could not apply ${DV_PROCESSING_EVENT_RINGBUFFER_PATCH}. "
      "The vendored dv-processing source may have changed; update the patch or reset the submodule "
      "with: git -C thirdparty/dv-processing checkout -- include/dv-processing/io/camera/camera_input_base.hpp")
  endif()
endif()

# Add dv-processing as submodule
add_subdirectory(thirdparty/dv-processing EXCLUDE_FROM_ALL)

# ROS2 + dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(dvs_msgs REQUIRED)  # EventArray message
find_package(OpenCV REQUIRED)
find_package(ZLIB REQUIRED)

# Executable definition
add_executable(dv_processing_driver_node
  src/driver_node.cpp
  src/compressed_encoder.cpp # this makes the encoding logic work
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
  ${event_camera_msgs_TARGETS}
)

# Ament dependencies (proper ROS include dirs & linking)
ament_target_dependencies(dv_processing_driver_node
  rclcpp
  sensor_msgs
  std_msgs
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
