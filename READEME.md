
# LTU_RAI | TNTX | Digitizing the ski environment 

This GitHub repository contains open source packages as well as developments on top of the open source resources as part of development efforts within the LTU_RAI | TNTX project.

The developments have been done using Robot Operating System (ROS) based frameworks. Therefore, running the packages and codes require some hardware as well as software requirements. 

The hardware set up for data collection is combination of different sensors, flight computer as well as computational unit as described below. 

## Hardware requirements

- Intel NUC i5 computational unit
- Vectornav IMU
- Velodyne VLP 16 PUCK Lite 3D LiDAR
- Power supply (LiHV/LiPo battery or a Power bank)
- Realsense D455 stereo camera

The hardware integration is completely custom and is inspired from one of the drones (Shafter) developed by Robotics and AI team of LTU.

## Software requirements

Ubuntu 18.04 installed on the Intel NUC or equivalent computational unit
ROS Melodic 
- [Ubuntu 18.04] (https://releases.ubuntu.com/18.04/)
- [Ros Melodic] (http://wiki.ros.org/melodic/Installation/Ubuntu)
    This should install all ROS related dependencies.
- [Computer Vision Liberary OpenCV](http://opencv.org/) 
- [C++ Library boost](http://www.boost.org/) 
- Velodyne drivers for ROS (http://wiki.ros.org/velodyne_driver)
- Realsense ROS (http://wiki.ros.org/RealSense)
- Vectornav ROS driver (https://github.com/dawonn/vectornav.git)
- [LIO_SAM] Lidar Inertial Odometry (https://github.com/TixiaoShan/LIO-SAM.git) (Source of odometry)

## Workspace

In a new terminal,

mkdir catkin_ws
cd catkin_ws/
mkdir src

cd catkin_ws/src

Install the related packages obj_localizer, darknet_ros, vision_opencv etc in the src folder of catkin_ws

## Data collection 

Install the Velodyne, Nevtornav and Realsense ros packages and LIO_SAM packages.

Run the corresponding launch files from above packages. 

In a new terminal 

cd catkin_ws/src
mkdir rosbags
cd rosbags/
rostopic record -a

When finished kill all terminals and take back up of the rosbag if it is too large. (save some time in object detection and localization process)

## Using the collected data

In a terminal run roscore

In a new terminal, 
cd catkin_ws/src
source devel/setup.bash
roslaunch darknet_ros yolo_v4_tiny.launch 

In a new terminal, 
cd catkin_ws/src
source devel/setup.bash
rosrun obj_localizer object_localizer.cpp 

In a new terminal,
cd catkin_ws/src
source devel/setup.bash
roslaunch visualization_tools visualize.launch

In a new terminal,
cd catkin_ws/src
source devel/setup.bash
rviz

In a new terminal,
cd catkin_ws/src/rosbags/
source devel/setup.bash
rosbag play "name_of_rosbag".bag










