#include "visualization_tools/visualize_navigation.hpp"

// STD
#include <string>

namespace visualization_tools
{

  VisualizeNavigation::VisualizeNavigation(ros::NodeHandle &nodeHandle)
      : nodeHandle_(nodeHandle)
  {

    odom_subscriber_ = nodeHandle_.subscribe("/something/odometry/imu", 1,
                                             &VisualizeNavigation::OdometryCallback, this);
    gt_traj_pub_ = nodeHandle_.advertise<nav_msgs::Path>("hummingbird/gt_path", 1);

    odom_noise_subscriber_ = nodeHandle_.subscribe("/something/noisevalue", 1,
                                                   &VisualizeNavigation::NoiseOdometryCallback, this);
    noise_traj_pub_ = nodeHandle_.advertise<nav_msgs::Path>("/noise_path", 1);

    waypoints_subscriber_ = nodeHandle_.subscribe("/something/reference", 1,
                                                  &VisualizeNavigation::WaypointsCallback, this);

    laserscan_subscriber_ = nodeHandle_.subscribe("/something/velodyne_points", 1,
                                                  &VisualizeNavigation::LaserCallback, this);

    waypoints_marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/waypoints", 1);
    ROS_INFO("Successfully launched node.");

    noise_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/noise_waypoints", 1);
    ROS_INFO("Successfully launched node.");

    mesh_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/quad_mesh", 1);
    //ROS_INFO("Successfully launched node.");
    mesh3_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/velodyne_mesh1", 1);
    //ROS_INFO("Successfully launched node.");
    mesh4_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/velodyne_mesh2", 1);
    //ROS_INFO("Successfully launched node.");
    mesh5_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/velodyne_mesh3", 1);
    //ROS_INFO("Successfully launched node.");
    mesh6_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/d455", 1);

    mesh7_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/prop1", 1);

    mesh8_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/prop2", 1);

    mesh9_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/prop3", 1);

    mesh10_marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/prop4", 1);
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

    //transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
    //transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

    visualization_msgs::Marker marker_msg1;

    marker_msg1.header.frame_id = "/world";
    marker_msg1.header.stamp = ros::Time::now();
    marker_msg1.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg1.mesh_resource = "package://visualization_tools/meshes/shafter_base.dae";
    marker_msg1.action = visualization_msgs::Marker::ADD;
    marker_msg1.mesh_use_embedded_materials = false;
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

    /*visualization_msgs::Marker marker_msg3;

    marker_msg3.header.frame_id = "/base_link";
    marker_msg3.header.stamp = ros::Time::now();
    marker_msg3.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg3.mesh_resource = "package://visualization_tools/meshes/VLP16_base_1.dae";
    marker_msg3.action = visualization_msgs::Marker::ADD;
    marker_msg3.mesh_use_embedded_materials = false;
    //marker_msg3.id = id;
    marker_msg3.pose.position.x = 0.01;
    marker_msg3.pose.position.y = 0.005;
    marker_msg3.pose.position.z = 0.077;
    marker_msg3.pose.orientation.x = 0;
    marker_msg3.pose.orientation.y = 0;
    marker_msg3.pose.orientation.z = 0;
    marker_msg3.pose.orientation.w = 1;
    marker_msg3.scale.x = 1.0;
    marker_msg3.scale.y = 1.0;
    marker_msg3.scale.z = 1.0;
    marker_msg3.color.a = 1.0;
    marker_msg3.color.r = 1.0;
    marker_msg3.color.g = 1.0;
    marker_msg3.color.b = 1.0;

    mesh3_marker_pub_.publish(marker_msg3);

    visualization_msgs::Marker marker_msg4;

    marker_msg4.header.frame_id = "/base_link";
    marker_msg4.header.stamp = ros::Time::now();
    marker_msg4.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg4.mesh_resource = "package://visualization_tools/meshes/VLP16_base_2.dae";
    marker_msg4.action = visualization_msgs::Marker::ADD;
    marker_msg4.mesh_use_embedded_materials = false;
    //marker_msg4.id = id;
    marker_msg4.pose.position.x = 0.01;
    marker_msg4.pose.position.y = 0.005;
    marker_msg4.pose.position.z = 0.077;
    marker_msg4.pose.orientation.x = 0;
    marker_msg4.pose.orientation.y = 0;
    marker_msg4.pose.orientation.z = 0;
    marker_msg4.pose.orientation.w = 1;
    marker_msg4.scale.x = 1.0;
    marker_msg4.scale.y = 1.0;
    marker_msg4.scale.z = 1.0;
    marker_msg4.color.a = 1.0;
    marker_msg4.color.r = 1.0;
    marker_msg4.color.g = 1.0;
    marker_msg4.color.b = 1.0;

    mesh4_marker_pub_.publish(marker_msg4);

    visualization_msgs::Marker marker_msg5;

    marker_msg5.header.frame_id = "/base_link";
    marker_msg5.header.stamp = ros::Time::now();
    marker_msg5.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg5.mesh_resource = "package://visualization_tools/meshes/VLP16_scan.dae";
    marker_msg5.action = visualization_msgs::Marker::ADD;
    marker_msg5.mesh_use_embedded_materials = false;
    //marker_msg5.id = id;
    marker_msg5.pose.position.x = 0.01;
    marker_msg5.pose.position.y = 0.005;
    marker_msg5.pose.position.z = 0.077;
    marker_msg5.pose.orientation.x = 0;
    marker_msg5.pose.orientation.y = 0;
    marker_msg5.pose.orientation.z = 0;
    marker_msg5.pose.orientation.w = 1;
    marker_msg5.scale.x = 1.0;
    marker_msg5.scale.y = 1.0;
    marker_msg5.scale.z = 1.0;
    marker_msg5.color.a = 1.0;
    marker_msg5.color.r = 1.0;
    marker_msg5.color.g = 1.0;
    marker_msg5.color.b = 1.0;

    mesh5_marker_pub_.publish(marker_msg5);

    visualization_msgs::Marker marker_msg6;

    marker_msg6.header.frame_id = "/base_link";
    marker_msg6.header.stamp = ros::Time::now();
    marker_msg6.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg6.mesh_resource = "package://visualization_tools/meshes/d435.dae";
    marker_msg6.action = visualization_msgs::Marker::ADD;
    marker_msg6.mesh_use_embedded_materials = false;
    //marker_msg6.id = id;
    marker_msg6.pose.position.x = 0.141;
    marker_msg6.pose.position.y = 0;
    marker_msg6.pose.position.z = -0.045;
    marker_msg6.pose.orientation.x = -0.4999998;
    marker_msg6.pose.orientation.y = 0.4996018;
    marker_msg6.pose.orientation.z = -0.4999998;
    marker_msg6.pose.orientation.w = 0.5003982;
    marker_msg6.scale.x = 1.0;
    marker_msg6.scale.y = 1.0;
    marker_msg6.scale.z = 1.0;
    marker_msg6.color.a = 1.0;
    marker_msg6.color.r = 1.0;
    marker_msg6.color.g = 1.0;
    marker_msg6.color.b = 1.0;
    mesh6_marker_pub_.publish(marker_msg6);

    /*
    visualization_msgs::Marker marker_msg7;

    marker_msg7.header.frame_id = "/world";
    marker_msg7.header.stamp = ros::Time::now();
    marker_msg7.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg7.mesh_resource = "package://visualization_tools/meshes/shafter_body_V2.dae";
    marker_msg7.action = visualization_msgs::Marker::ADD;
    marker_msg7.mesh_use_embedded_materials = false;
    //marker_msg7.id = id;
    marker_msg7.pose.position.x = msg.pose.pose.position.x;
    marker_msg7.pose.position.y = msg.pose.pose.position.y;
    marker_msg7.pose.position.z = msg.pose.pose.position.z;
    marker_msg7.pose.orientation.x = msg.pose.pose.orientation.x;
    marker_msg7.pose.orientation.y = msg.pose.pose.orientation.y;
    marker_msg7.pose.orientation.z = msg.pose.pose.orientation.z;
    marker_msg7.pose.orientation.w = msg.pose.pose.orientation.w;
    marker_msg7.scale.x = 1.0;
    marker_msg7.scale.y = 1.0;
    marker_msg7.scale.z = 1.0;
    marker_msg7.color.a = 1.0;
    marker_msg7.color.r = 1.0;
    marker_msg7.color.g = 1.0;
    marker_msg7.color.b = 1.0;
    mesh7_marker_pub_.publish(marker_msg7);


    visualization_msgs::Marker marker_msg8;

    marker_msg8.header.frame_id = "/world";
    marker_msg8.header.stamp = ros::Time::now();
    marker_msg8.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg8.mesh_resource = "package://visualization_tools/meshes/shafter_body_V2.dae";
    marker_msg8.action = visualization_msgs::Marker::ADD;
    marker_msg8.mesh_use_embedded_materials = false;
    //marker_msg8.id = id;
    marker_msg8.pose.position.x = msg.pose.pose.position.x;
    marker_msg8.pose.position.y = msg.pose.pose.position.y;
    marker_msg8.pose.position.z = msg.pose.pose.position.z;
    marker_msg8.pose.orientation.x = msg.pose.pose.orientation.x;
    marker_msg8.pose.orientation.y = msg.pose.pose.orientation.y;
    marker_msg8.pose.orientation.z = msg.pose.pose.orientation.z;
    marker_msg8.pose.orientation.w = msg.pose.pose.orientation.w;
    marker_msg8.scale.x = 1.0;
    marker_msg8.scale.y = 1.0;
    marker_msg8.scale.z = 1.0;
    marker_msg8.color.a = 1.0;
    marker_msg8.color.r = 1.0;
    marker_msg8.color.g = 1.0;
    marker_msg8.color.b = 1.0;
    mesh8_marker_pub_.publish(marker_msg8);


    visualization_msgs::Marker marker_msg9;

    marker_msg9.header.frame_id = "/world";
    marker_msg9.header.stamp = ros::Time::now();
    marker_msg9.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg9.mesh_resource = "package://visualization_tools/meshes/shafter_body_V2.dae";
    marker_msg9.action = visualization_msgs::Marker::ADD;
    marker_msg9.mesh_use_embedded_materials = false;
    //marker_msg9.id = id;
    marker_msg9.pose.position.x = msg.pose.pose.position.x;
    marker_msg9.pose.position.y = msg.pose.pose.position.y;
    marker_msg9.pose.position.z = msg.pose.pose.position.z;
    marker_msg9.pose.orientation.x = msg.pose.pose.orientation.x;
    marker_msg9.pose.orientation.y = msg.pose.pose.orientation.y;
    marker_msg9.pose.orientation.z = msg.pose.pose.orientation.z;
    marker_msg9.pose.orientation.w = msg.pose.pose.orientation.w;
    marker_msg9.scale.x = 1.0;
    marker_msg9.scale.y = 1.0;
    marker_msg9.scale.z = 1.0;
    marker_msg9.color.a = 1.0;
    marker_msg9.color.r = 1.0;
    marker_msg9.color.g = 1.0;
    marker_msg9.color.b = 1.0;
    mesh9_marker_pub_.publish(marker_msg9);
    
    
    
    visualization_msgs::Marker marker_msg10;

    marker_msg10.header.frame_id = "/world";
    marker_msg10.header.stamp = ros::Time::now();
    marker_msg10.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg10.mesh_resource = "package://visualization_tools/meshes/shafter_body_V2.dae";
    marker_msg10.action = visualization_msgs::Marker::ADD;
    marker_msg10.mesh_use_embedded_materials = false;
    //marker_msg10.id = id;
    marker_msg10.pose.position.x = msg.pose.pose.position.x;
    marker_msg10.pose.position.y = msg.pose.pose.position.y;
    marker_msg10.pose.position.z = msg.pose.pose.position.z;
    marker_msg10.pose.orientation.x = msg.pose.pose.orientation.x;
    marker_msg10.pose.orientation.y = msg.pose.pose.orientation.y;
    marker_msg10.pose.orientation.z = msg.pose.pose.orientation.z;
    marker_msg10.pose.orientation.w = msg.pose.pose.orientation.w;
    marker_msg10.scale.x = 1.0;
    marker_msg10.scale.y = 1.0;
    marker_msg10.scale.z = 1.0;
    marker_msg10.color.a = 1.0;
    marker_msg10.color.r = 1.0;
    marker_msg10.color.g = 1.0;
    marker_msg10.color.b = 1.0;
    mesh10_marker_pub_.publish(marker_msg10);*/
  }

  void VisualizeNavigation::NoiseOdometryCallback(const nav_msgs::Odometry &msg)
  {
    /*
  counter_cleaner++;
  if (counter_cleaner > 100)
  {
    path_noise.poses.clear();
    counter_cleaner = 0;
  }
  geometry_msgs::PoseStamped noise_cur_pose;
  noise_cur_pose.header.stamp = ros::Time::now();
  noise_cur_pose.pose = msg.pose.pose;
  noise_cur_pose.pose.position.z = cur_pose.pose.position.z;
  path_noise.header.frame_id = "/world";
  path_noise.poses.push_back(noise_cur_pose);
  noise_traj_pub_.publish(path_noise); */

    marker_msg_noise.header.frame_id = "/world";
    marker_msg_noise.header.stamp = ros::Time::now();
    marker_msg_noise.type = visualization_msgs::Marker::CUBE;
    marker_msg_noise.action = visualization_msgs::Marker::ADD;
    marker_msg_noise.id = id;
    marker_msg_noise.pose.position.x = msg.pose.pose.position.x;
    marker_msg_noise.pose.position.y = msg.pose.pose.position.y;
    marker_msg_noise.pose.position.z = msg.pose.pose.position.z;
    marker_msg_noise.pose.orientation.x = 0.0;
    marker_msg_noise.pose.orientation.y = 0.0;
    marker_msg_noise.pose.orientation.z = 0.0;
    marker_msg_noise.pose.orientation.w = 1.0;
    marker_msg_noise.scale.x = 0.3;
    marker_msg_noise.scale.y = 0.3;
    marker_msg_noise.scale.z = 0.3;
    marker_msg_noise.color.a = 1.0;
    marker_msg_noise.color.r = 1.0;
    marker_msg_noise.color.g = 0.0;
    marker_msg_noise.color.b = 0.0;

    noise_marker_pub_.publish(marker_msg_noise);
  }

  void VisualizeNavigation::WaypointsCallback(const nav_msgs::Odometry &msg)
  {
    visualization_msgs::Marker marker_msg;

    marker_msg.header.frame_id = "/world";
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.type = visualization_msgs::Marker::CUBE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.id = id;
    marker_msg.pose.position.x = msg.pose.pose.position.x;
    marker_msg.pose.position.y = msg.pose.pose.position.y;
    marker_msg.pose.position.z = msg.pose.pose.position.z;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.15;
    marker_msg.scale.y = 0.15;
    marker_msg.scale.z = 0.15;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    //std::cout << "counter: " << counter << std::endl;

    if (counter > 10)
    {
      counter = 0;
      marker_array_msg.markers.push_back(marker_msg);
      //std::cout << "marker_array_msg.markers.size(): " << marker_array_msg.markers.size() << std::endl;
    }
    for (int i = 0; i < marker_array_msg.markers.size(); i++)
    {
      waypoints_marker_pub_.publish(marker_array_msg);
    }
    counter++;
    id++;
  }

  void VisualizeNavigation::LaserCallback(const sensor_msgs::LaserScan &scan_msg)
  {
    std::vector<double> ranges;
    //counter_test = 0;
    //std::cout << "ranges.size(): " << scan_msg.ranges.size() << std::endl;
    for (int i = 0; i < scan_msg.ranges.size(); i++)
    {
      if (scan_msg.ranges[i] != inf)
      {
        ranges.push_back(scan_msg.ranges[i]);
        //std::cout << "ranges: " << scan_msg.ranges[i] << std::endl;
        //std::cout << "counter_test: " << counter_test << std::endl;
        //counter_test++;
      }
    }

    //std::cout << "obs_back: " << ranges[0] << std::endl;
    //std::cout << "obs_left: " << ranges[339] << std::endl;
    //std::cout << "obs_front: " << ranges[223] << std::endl;
    //std::cout << "obs_right: " << ranges[90] << std::endl;

    // Distance from obstacle in front
    obs_back = ranges[0];
    // Distance from obstacle on the back
    obs_front = ranges[223];
    // Distance from obstacle on the left
    obs_left = ranges[339];
    // Distance from obstacle on the back
    obs_right = ranges[90];
  }

} // namespace visualization_tools
