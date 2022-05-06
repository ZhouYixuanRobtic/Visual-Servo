cmake_minimum_required(VERSION 2.8.3)
project(visual_servo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport
                interactive_markers moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface
                pluginlib geometric_shapes pcl_ros pcl_conversions rosbag tf2_ros tf2_eigen tf2_geometry_msgs message_generation cartographer_ros_msgs)

find_package(
        OpenCV REQUIRED
)

find_package(Boost REQUIRED system filesystem date_time thread)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED yaml-cpp)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
  TagsDetection_msg.msg
   TagDetection_msg.msg
        VisualServoMetaTypeMsg.msg
 )
## Generate services in the 'srv' folder
add_service_files(
        FILES
        detect_once.srv
        manipulate.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )



catkin_package(
        # INCLUDE_DIRS src LIBRARIES KeyboardTeleop
        # INCLUDE_DIRS src LIBRARIES ControlTeleop
        # INCLUDE_DIRS src LIBRARIES SerialManager
  INCLUDE_DIRS include
  LIBRARIES
        KeyboardTeleop
        ControlTeleop
        SerialManager
  CATKIN_DEPENDS
                 cv_bridge
                 std_msgs
                 sensor_msgs
                 image_transport
                 roscpp
                 geometry_msgs
                 moveit_core
                 moveit_visual_tools
                 moveit_ros_planning_interface
                 interactive_markers
                 tf2_ros tf2_eigen tf2_geometry_msgs
                 message_runtime
        DEPENDS
                 EIGEN3
                 YAML
                 Boost
)

set(AUBO_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/dependents/robotSDK/inc;
                        ${CMAKE_CURRENT_SOURCE_DIR}/include/example/)
set(AUBO_LINK_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/dependents/protobuf/linux-x64/lib;
                ${CMAKE_CURRENT_SOURCE_DIR}/include/dependents/robotController/lib-linux64;
                ${CMAKE_CURRENT_SOURCE_DIR}/include/dependents/log4cplus/linux_x64/lib;
                ${CMAKE_CURRENT_SOURCE_DIR}/include/dependents/libconfig/linux_x64/lib;
                ${CMAKE_CURRENT_SOURCE_DIR}/include/dependents/robotSDK/lib/linux_x64)

set(VISUAL_SERVO_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/visual_servo/)
include_directories (
        SYSTEM
        include
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        /usr/local/include/tf
        /usr/local/include/apriltag
        /usr/local/include/sophus
        ${catkin_INCLUDE_DIRS}
        ${AUBO_INCLUDE_PATH}
        ${YAML_INCLUDE_DIRS}
        ${VISUAL_SERVO_INCLUDE_PATH}
)
link_directories(
        ${OpenCV_LIB_DIR}
        ${catkin_LIBRARY_DIRS}
        /usr/local/lib
        ${AUBO_LINK_DIRS}
        ${YAML_LIBRARIES}
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/visual_servo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#libraries

add_library(auboUtil include/example/util.cpp)
target_link_libraries(auboUtil protobuf our_alg_i5p log4cplus config auborobotcontroller)

add_library(auboA src/AuboSDK.cpp)
target_link_libraries(auboA auboUtil protobuf our_alg_i5p log4cplus config auborobotcontroller )
add_dependencies(auboA ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_library(servo src/Servo.cpp)
target_link_libraries(servo apriltag ${OpenCV_LIBRARIES} Sophus)
add_dependencies(servo ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_library(detector src/Detector.cpp src/radon_transform.cpp)
target_link_libraries(detector ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_library(KeyboardTeleop src/KeyboardTeleop.cpp)
target_link_libraries(KeyboardTeleop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(parameterTeleop src/parameterTeleop.cpp)
target_link_libraries(parameterTeleop ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_library(USBCamera src/USBCamera.cpp)
target_link_libraries(USBCamera ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(USBCamera ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_library(ControlTeleop src/ControlTeleop.cpp)
target_link_libraries(ControlTeleop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(SerialManager src/SerialManager.cpp)
target_link_libraries(SerialManager ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(ManiSerialManager src/ManiSerialManager.cpp)
target_link_libraries(ManiSerialManager ${catkin_LIBRARIES} ${Boost_LIBRARIES} SerialManager)

#add_library(BLManager src/BLManager.cpp)
#target_link_libraries(BLManager ${catkin_LIBRARIES} ${Boost_LIBRARIES} bluetooth)
#application
add_executable(auboRobot src/auboRobotLaunch.cpp)
target_link_libraries(auboRobot auboA protobuf our_alg_i5p log4cplus config auborobotcontroller auboUtil ${catkin_LIBRARIES} ${Boost_LIBRARIES} KeyboardTeleop parameterTeleop ControlTeleop)

add_executable(visual_servo src/main.cpp)
target_link_libraries(visual_servo ${catkin_LIBRARIES} apriltag servo ${OpenCV_LIBRARIES} Sophus ${Boost_LIBRARIES} ${YAML_LIBRARIES} parameterTeleop ManiSerialManager)
add_dependencies(visual_servo ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(real_sense src/camera.cpp)
target_link_libraries(real_sense detector ${catkin_LIBRARIES} servo ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${YAML_LIBRARIES})
add_dependencies(real_sense ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(monocular src/newCamera.cpp)
target_link_libraries(monocular detector ${catkin_LIBRARIES} servo ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${YAML_LIBRARIES} USBCamera)
add_dependencies(monocular ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(newtest test/newtest.cpp)
target_link_libraries(newtest detector ${OpenCV_LIBS} ${catkin_LIBRARIES} )

add_executable(maTest test/maTest.cpp)
target_link_libraries(maTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} KeyboardTeleop ControlTeleop ${YAML_LIBRARIES})
add_dependencies(maTest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(keyTest test/keyboardTest.cpp)
target_link_libraries(keyTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} KeyboardTeleop)

add_executable(toolTest test/toolTest.cpp)
target_link_libraries(toolTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} KeyboardTeleop ControlTeleop)

add_executable(landmark_broadcaster src/landmark_broadcaster.cpp)
target_link_libraries(landmark_broadcaster servo ${catkin_LIBRARIES})

#add_executable(BLRosWrapper src/BLROSWrapper.cpp)
#target_link_libraries(BLRosWrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES} BLManager)
#add_dependencies(BLRosWrapper ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

#add_executable(blTest test/BLTest.cpp)
#target_link_libraries(blTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} KeyboardTeleop ControlTeleop ${YAML_LIBRARIES} bluetooth)
#add_dependencies(blTest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
