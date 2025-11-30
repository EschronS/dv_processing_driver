cmake_minimum_required(VERSION 3.14)
project(dv_processing_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(OpenCV REQUIRED)

# Add dv-processing
add_subdirectory(thirdparty/dv-processing EXCLUDE_FROM_ALL)

# Your driver executable or library
add_executable(driver src/driver.cpp)

# Include OpenCV headers
target_include_directories(driver PUBLIC ${OpenCV_INCLUDE_DIRS})

# Link your driver against dv-processing
target_link_libraries(driver
    PRIVATE
        dv::processing
        ${OpenCV_LIBS}
)
