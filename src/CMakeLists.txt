cmake_minimum_required(VERSION 3.13)
project(Nursing_robot)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-std=c++14 -lboost_system)
find_package(
        OpenCV REQUIRED
)

include_directories (
        "/home/xcy/module/OpenNI-Linux-x64-2.2/Include"
        "/home/xcy/module/NiTE-2.0.0/Include"
        ${OpenCV_INCLUDE_DIRS}
        /usr/local/include/tf
        /usr/local/include/apriltag
        KCFcpp
        /usr/local/include/sophus
)
link_directories(
        "/home/xcy/module/OpenNI-Linux-x64-2.2/Redist"
        "/home/xcy/module/NiTE-2.0.0/Redist"
        ${OpenCV_LIB_DIR}
        /user/local/lib
)
FILE(GLOB_RECURSE sourcefiles "KCFcpp/*.cpp")
add_library(KCF ${sourcefiles} )
target_link_libraries(KCF ${OpenCV_LIBS})

add_library(servo Servo.cpp)
target_link_libraries(servo apriltag KCF ${OpenCV_LIBS} OpenNI2 NiTE2 Sophus)

add_library(draw Draw.cpp)
target_link_libraries(draw ${OpenCV_LIBS})
add_library(camera camera.cpp)
target_link_libraries(camera ${OpenCV_LIBS} OpenNI2 NiTE2 draw)

add_executable(Nursing_robot main.cpp)
target_link_libraries(Nursing_robot camera apriltag servo KCF ${OpenCV_LIBS} OpenNI2 NiTE2 Sophus)
