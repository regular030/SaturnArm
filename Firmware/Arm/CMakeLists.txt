cmake_minimum_required(VERSION 3.10)
project(VR_Arm_Control)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(vr_arm_control
    server.cpp
    camera.cpp
    arm_controller.cpp
)

target_link_libraries(vr_arm_control
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    pthread
    pigpiod_if2
)