#include "visualization_tools/visualize_navigation.hpp"

// STD
#include <string>

namespace visualization_tools
{

VisualizeNavigation::VisualizeNavigation(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{

  odom_subscriber_ = nodeHandle_.subscribe("/pixy/truth/NWU", 1,
                                           &VisualizeNavigation::OdometryCallback, this);
  gt_traj_pub_ = nodeHandle_.advertise<nav_msgs::Path>("/gt_path1", 1);

  

  mesh_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/quad_mesh", 1);
  ROS_INFO("Successfully launched node.");
  mesh3_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/velodyne_mesh1", 1);
  ROS_INFO("Successfully launched node.");
  mesh4_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/velodyne_mesh2", 1);
  ROS_INFO("Successfully launched node.");
  mesh5_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/velodyne_mesh3", 1);
  ROS_INFO("Successfully launched node.");
}

VisualizeNavigation::~VisualizeNavigation()
{
}

void VisualizeNavigation::OdometryCallback(const nav_msgs::Odometry &msg)
{
  cur_pose.header.stamp = ros::Time::now();
  cur_pose.pose = msg.pose.pose;
  path_gt.header.frame_id = "/world";
  path_gt.poses.push_back(cur_pose);
  gt_traj_pub_.publish(path_gt);

  //publish tfs

  //tf::Quaternion q;
  //q.setRPY(-1.57, 1.57, 0);
  //transform.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tag_ENU", "tag_15"));

  transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pixy"));
  
  visualization_msgs::Marker marker_msg1;

  marker_msg1.header.frame_id = "/world";
  marker_msg1.header.stamp = ros::Time::now();
  marker_msg1.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_msg1.mesh_resource = "package://visualization_tools/meshes/quadrotor_base.dae";
  marker_msg1.action = visualization_msgs::Marker::ADD;
  marker_msg1.mesh_use_embedded_materials=false;
  //marker_msg1.id = id;
  marker_msg1.pose.position.x = msg.pose.pose.position.x;
  marker_msg1.pose.position.y = msg.pose.pose.position.y;
  marker_msg1.pose.position.z = msg.pose.pose.position.z;
  marker_msg1.pose.orientation.x = msg.pose.pose.orientation.x;
  marker_msg1.pose.orientation.y = msg.pose.pose.orientation.y;
  marker_msg1.pose.orientation.z = msg.pose.pose.orientation.z;
  marker_msg1.pose.orientation.w = msg.pose.pose.orientation.w;
  marker_msg1.scale.x = 1.0;
  marker_msg1.scale.y = 1.0;
  marker_msg1.scale.z = 1.0;
  marker_msg1.color.a = 1.0;
  marker_msg1.color.r = 1.0;
  marker_msg1.color.g = 1.0;
  marker_msg1.color.b = 1.0;
  mesh_marker_pub_.publish(marker_msg1);


visualization_msgs::Marker marker_msg3;

  marker_msg3.header.frame_id = "/world";
  marker_msg3.header.stamp = ros::Time::now();
  marker_msg3.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_msg3.mesh_resource = "package://visualization_tools/meshes/VLP16_base_1.dae";
  marker_msg3.action = visualization_msgs::Marker::ADD;
  marker_msg3.mesh_use_embedded_materials=false;
  //marker_msg3.id = id;
  marker_msg3.pose.position.x = msg.pose.pose.position.x;
  marker_msg3.pose.position.y = msg.pose.pose.position.y;
  marker_msg3.pose.position.z = msg.pose.pose.position.z+0.08;
  marker_msg3.pose.orientation.x = msg.pose.pose.orientation.x;
  marker_msg3.pose.orientation.y = msg.pose.pose.orientation.y;
  marker_msg3.pose.orientation.z = msg.pose.pose.orientation.z;
  marker_msg3.pose.orientation.w = msg.pose.pose.orientation.w;
  marker_msg3.scale.x = 1.0;
  marker_msg3.scale.y = 1.0;
  marker_msg3.scale.z = 1.0;
  marker_msg3.color.a = 1.0;
  marker_msg3.color.r = 1.0;
  marker_msg3.color.g = 1.0;
  marker_msg3.color.b = 1.0;

  mesh3_marker_pub_.publish(marker_msg3);


  visualization_msgs::Marker marker_msg4;

  marker_msg4.header.frame_id = "/world";
  marker_msg4.header.stamp = ros::Time::now();
  marker_msg4.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_msg4.mesh_resource = "package://visualization_tools/meshes/VLP16_base_2.dae";
  marker_msg4.action = visualization_msgs::Marker::ADD;
  marker_msg4.mesh_use_embedded_materials=false;
  //marker_msg4.id = id;
  marker_msg4.pose.position.x = msg.pose.pose.position.x;
  marker_msg4.pose.position.y = msg.pose.pose.position.y;
  marker_msg4.pose.position.z = msg.pose.pose.position.z+0.08;
  marker_msg4.pose.orientation.x = msg.pose.pose.orientation.x;
  marker_msg4.pose.orientation.y = msg.pose.pose.orientation.y;
  marker_msg4.pose.orientation.z = msg.pose.pose.orientation.z;
  marker_msg4.pose.orientation.w = msg.pose.pose.orientation.w;
  marker_msg4.scale.x = 1.0;
  marker_msg4.scale.y = 1.0;
  marker_msg4.scale.z = 1.0;
  marker_msg4.color.a = 1.0;
  marker_msg4.color.r = 1.0;
  marker_msg4.color.g = 1.0;
  marker_msg4.color.b = 1.0;

  mesh4_marker_pub_.publish(marker_msg4);

  visualization_msgs::Marker marker_msg5;

  marker_msg5.header.frame_id = "/world";
  marker_msg5.header.stamp = ros::Time::now();
  marker_msg5.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_msg5.mesh_resource = "package://visualization_tools/meshes/VLP16_scan.dae";
  marker_msg5.action = visualization_msgs::Marker::ADD;
  marker_msg5.mesh_use_embedded_materials=false;
  //marker_msg5.id = id;
  marker_msg5.pose.position.x = msg.pose.pose.position.x;
  marker_msg5.pose.position.y = msg.pose.pose.position.y;
  marker_msg5.pose.position.z = msg.pose.pose.position.z+0.08;
  marker_msg5.pose.orientation.x = msg.pose.pose.orientation.x;
  marker_msg5.pose.orientation.y = msg.pose.pose.orientation.y;
  marker_msg5.pose.orientation.z = msg.pose.pose.orientation.z;
  marker_msg5.pose.orientation.w = msg.pose.pose.orientation.w;
  marker_msg5.scale.x = 1.0;
  marker_msg5.scale.y = 1.0;
  marker_msg5.scale.z = 1.0;
  marker_msg5.color.a = 1.0;
  marker_msg5.color.r = 1.0;
  marker_msg5.color.g = 1.0;
  marker_msg5.color.b = 1.0;

  mesh5_marker_pub_.publish(marker_msg5);


}



} // namespace visualization_tools
