cmake_minimum_required(VERSION 3.12)
project(SaturnArm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# Add executable
add_executable(SaturnArm
    server.cpp
    arm_controller.cpp
    camera.cpp
    wifi_util.cpp
)

# Link libraries
target_link_libraries(SaturnArm
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    pigpiod_if2
    pthread
    wiringPi
)

# Install target
install(TARGETS SaturnArm RUNTIME DESTINATION bin)
