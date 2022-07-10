## Object localizer ROS package

This is a object localization implementation that holds instantaneous position of the detected bounding box in the global coordinate system. 

## Installation and Building

cd catkin_ws/src

Clone the package 

cd ..

catkin build obj_localizer

source devel/setup.bash

## Running the package

rosrun obj_localizer object_localizer.cpp

## Visualization

rqt_image_view

