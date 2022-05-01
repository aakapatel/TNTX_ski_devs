#pragma once

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <tf/transform_broadcaster.h>

namespace visualization_tools
{

class VisualizeNavigation3
{
public:
  VisualizeNavigation3(ros::NodeHandle &nodeHandle);

  virtual ~VisualizeNavigation3();

private:
  tf::TransformBroadcaster br;
  tf::Transform transform;

  double inf = std::numeric_limits<double>::infinity();
  int counter = 0;
  int counter_test = 0;
  int counter_cleaner = 0;
  int id = 0;
  double obs_front, obs_back, obs_left, obs_right;
  nav_msgs::Path path_gt;
  nav_msgs::Path path_noise;
  visualization_msgs::MarkerArray waypoints;
  geometry_msgs::PoseStamped cur_pose;
  visualization_msgs::MarkerArray marker_array_msg;
  visualization_msgs::Marker marker_msg_noise;

  void OdometryCallback(const nav_msgs::Odometry &msg);
  void NoiseOdometryCallback(const nav_msgs::Odometry &msg);
  void WaypointsCallback(const nav_msgs::Odometry &msg);
  void LaserCallback(const sensor_msgs::LaserScan &scan_msg);

  //! ROS node handle.
  ros::NodeHandle &nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber odom_subscriber_;
  ros::Subscriber odom_noise_subscriber_;
  ros::Subscriber waypoints_subscriber_;
  ros::Subscriber laserscan_subscriber_;

  ros::Publisher gt_traj_pub_;
  ros::Publisher noise_traj_pub_;
  ros::Publisher waypoints_marker_pub_;
  ros::Publisher noise_marker_pub_;
  ros::Publisher mesh_marker_pub_;
  ros::Publisher mesh3_marker_pub_;
  ros::Publisher mesh4_marker_pub_;
  ros::Publisher mesh5_marker_pub_;
  ros::Publisher mesh6_marker_pub_;
  ros::Publisher mesh7_marker_pub_;
  ros::Publisher mesh8_marker_pub_;
  ros::Publisher mesh9_marker_pub_;
  ros::Publisher mesh10_marker_pub_;
};

} // namespace visualization_tools
