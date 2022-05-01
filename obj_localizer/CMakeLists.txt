cmake_minimum_required(VERSION 2.8.3)
project(obj_localizer)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS geometry_msgs darknet_ros_msgs roscpp sensor_msgs cv_bridge std_msgs image_transport rospy tf tf2 tf_conversions visualization_msgs)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    darknet_ros_msgs
    roscpp
    sensor_msgs
    cv_bridge 
    std_msgs 
    image_transport
    message_filters
    rospy
    std_srvs
    tf
    tf2
    tf_conversions
    visualization_msgs
    # yolo_ros_vino
)

#find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_srvs
#)


catkin_package(
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
  #DEPENDS Eigen std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(object_localizer src/object_localizer.cpp)
target_link_libraries(object_localizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_dependencies(bounding_box_depth ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
