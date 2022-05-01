cmake_minimum_required(VERSION 2.8.3)
project(visualization_tools)

## Use C++11
add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf
    tf2
    cv_bridge
    image_transport
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
#  DEPENDS
#    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
)


## Declare cpp executables
add_executable(visualize_navigation
  src/visualize_navigation_node.cpp
  src/visualize_navigation.cpp

)


## Add dependencies to exported targets, like ROS msgs or srvs

add_dependencies(visualize_navigation
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against

target_link_libraries(visualize_navigation
  ${catkin_LIBRARIES}
)


## Declare cpp executables
add_executable(visualize_navigation1
  src/visualize_navigation_node.cpp
  src/visualize_navigation1.cpp

)

## Add dependencies to exported targets, like ROS msgs or srvs

add_dependencies(visualize_navigation1
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against

target_link_libraries(visualize_navigation1
  ${catkin_LIBRARIES}
)


add_executable(visualize_navigation2
  src/visualize_navigation_node2.cpp
  src/visualize_navigation2.cpp

)



add_dependencies(visualize_navigation2
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against

target_link_libraries(visualize_navigation2
  ${catkin_LIBRARIES}
)


add_executable(visualize_navigation3
  src/visualize_navigation_node3.cpp
  src/visualize_navigation3.cpp

)


add_dependencies(visualize_navigation3
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against

target_link_libraries(visualize_navigation3
  ${catkin_LIBRARIES}
)
