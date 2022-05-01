#include "Eigen/Core"
#include "Eigen/Geometry"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace sensor_msgs;
using namespace message_filters;
tf::TransformListener *tran;
std::string class_name;
visualization_msgs::MarkerArray marker_msg1;
std::list<geometry_msgs::Point> humans;
Eigen::Vector4d hom_loc;
Eigen::Vector4d hom_loc_temp;
geometry_msgs::Point human;
geometry_msgs::Point artifact_position;
double fx = 381.95654296875; // TODO read it either from camera_info topic or
                             // set through yaml
double fy = 381.4194030761719;
int cx = 317.76678466796875;
int cy = 248.4901123046875;
float value = 0, GrayValue = 0;
int ind = 0;
bool new_artifact = false;
ros::Time callback_time;
ros::Time callback_time_checkpoint;

visualization_msgs::MarkerArray marker_msg_array;
ros::Publisher artifact_marker_pub_;
ros::Publisher artifact_found_flag_pub_;

// function to compute average
Eigen::Vector3d compute_average(std::vector<Eigen::Vector3d> &vi) {

  double sumx = 0;
  double sumy = 0;
  double sumz = 0;

  for (int p = 0; p < vi.size(); p++) {

    sumx = sumx + vi.at(p)(0);
    sumy = sumy + vi.at(p)(1);
    sumz = sumz + vi.at(p)(2);
  }
  Eigen::Vector3d sumS;
  sumS(0) = sumx / vi.size();
  sumS(1) = sumy / vi.size();
  sumS(2) = sumz / vi.size();
  return sumS;
}

Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
  Eigen::Affine3d e;
  for (int i = 0; i < 3; i++) {
    e.matrix()(i, 3) = t.getOrigin()[i];
    for (int j = 0; j < 3; j++) {
      e.matrix()(i, j) = t.getBasis()[i][j];
    }
  }
  // Fill in identity in last row
  for (int col = 0; col < 3; col++)
    e.matrix()(3, col) = 0;
  e.matrix()(3, 3) = 1;
  return e;
}

void callback(const ImageConstPtr &image1_msg,
              const darknet_ros_msgs::BoundingBoxesConstPtr &boxes_msg,
              const nav_msgs::OdometryConstPtr &odom_msg) {

// std::cout<< "artifact.X: " << std::endl;

  // Solve all perception here...

  // proc depth image
  cv::Mat depthimage;
  GrayValue = 0;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image1_msg,
                                 sensor_msgs::image_encodings::TYPE_32FC1);
    cv_ptr->image.copyTo(depthimage);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
              image1_msg->encoding.c_str());
  }
  // std::cout << "here2" << std::endl;
  if (boxes_msg->bounding_boxes.size() != 0) {
    for (int art_i = 0; art_i < boxes_msg->bounding_boxes.size(); art_i++) {
      
        class_name = boxes_msg->bounding_boxes.at(art_i).Class;

        if (class_name == "person") {
      
      int xmin = boxes_msg->bounding_boxes.at(art_i).xmin;
      int xmax = boxes_msg->bounding_boxes.at(art_i).xmax;
      int ymin = boxes_msg->bounding_boxes.at(art_i).ymin;
      int ymax = boxes_msg->bounding_boxes.at(art_i).ymax;

      int u = int((xmin + xmax) / 2);
      int v = int((ymin + ymax) / 2);
      // std::cout << "here3" << std::endl;

      for (int i = u - 1; i <= u + 1; i++) {
        for (int j = v - 1; j <= v + 1; j++) {
          value = (float)depthimage.at<float>(j, i);

          if (value > 0) {
            GrayValue += value; // sum pixels around center
            ind++;
          }
        }
      }
      GrayValue = GrayValue / ind;
      ind = 0;
      double x = 0, y = 0, z = 0;
      // depth_cam proc!!
      z = double(GrayValue) / 1000;

      double artifact_width = 0;
      double m_xmin = ((xmin - cx) * z) / fx;
      double m_ymin = ((ymin - cy) * z) / fy;
      double m_xmax = ((xmax - cx) * z) / fx;
      double m_ymax = ((ymax - cy) * z) / fy;
      artifact_width = abs(m_xmax - m_xmin);
      if (artifact_width != NAN) {
        x = ((u - cx) * z) / fx;
        y = ((v - cy) * z) / fy;

        // artifacts....
        artifact_position.x = x;
        artifact_position.y = y;

        if ( z < 5 && z >= 0.1) {

          artifact_position.z = z;
        }

                if ( x < 5 && x >= 0.1) {

          artifact_position.x = x;
        }

                if ( y < 5 && y >= 0.1) {

          artifact_position.y = y;
        }

        // std::cout<< "artifact.X: " << x << std::endl;


        tf::StampedTransform transform;
        try {
          // tran->lookupTransform("/world", "/color",
          //                      ros::Time(0), transform);


          // configured for shafter for now
          tran->lookupTransform("/world_shafter", "/camera_color_optical_frame",
                                ros::Time(0.0), transform);
        } catch (tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
          ros::Duration(0.1).sleep();
        }
        Eigen::Affine3d test;
        test = transformTFToEigen(transform);
        hom_loc_temp(0) = artifact_position.x;
        hom_loc_temp(1) = artifact_position.y;
        hom_loc_temp(2) = artifact_position.z;
        hom_loc_temp(3) = 1;

        hom_loc = test.matrix() * hom_loc_temp;

        Eigen::Vector3d artifacts_;
        artifacts_(0) = hom_loc(0);
        artifacts_(1) = hom_loc(1);
        artifacts_(2) = hom_loc(2);

        human.x = artifacts_(0);
        human.y = artifacts_(1);
        human.z = artifacts_(2);

        humans.push_back(human);

        std::cout<< "artifact_loc: " << artifacts_(0) << ' ' << artifacts_(1) << ' ' << artifacts_(2) << std::endl;
        // std::cout<< "artifact.Y: " << artifacts_(1) << std::endl;
        // std::cout<< "artifact.Z: " << artifacts_(2) << std::endl;

        }

        // callback_time_checkpoint = ros::Time::now(); //this is for usage of the timer callback function

        //.push_back(artifacts_); //to stack more detections
      }
    }
  }
}

void callback2(const ros::TimerEvent &) {
  // ROS_INFO("Timer Callback triggered");
  callback_time = ros::Time::now();

  auto dt = callback_time - callback_time_checkpoint;
  // std::cout << "dt.toSec(): " << dt.toSec() << std::endl;

  // if (dt.toSec() < 10 && dt.toSec() >= 7) {

    // uncomment and modify for visualization if needed
    
        // if (new_artifact) {
        //   new_artifact = false;

        marker_msg1.markers.resize(humans.size());
        for (geometry_msgs::Point detected : humans) {

          for(unsigned i=0; i < marker_msg1.markers.size(); i++) {

          
          marker_msg1.markers[i].header.frame_id = "world_shafter";
          marker_msg1.markers[i].header.stamp = ros::Time::now();
          marker_msg1.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;
          marker_msg1.markers[i].action = visualization_msgs::Marker::ADD;
          marker_msg1.markers[i].mesh_use_embedded_materials = false;
          marker_msg1.markers[i].ns = i;
          marker_msg1.markers[i].id = i;
          marker_msg1.markers[i].scale.x = 0.4;
          marker_msg1.markers[i].scale.y = 0.4;
          marker_msg1.markers[i].scale.z = 0.4;
          marker_msg1.markers[i].color.a = 1.0;
          marker_msg1.markers[i].color.r = 0.0;
          marker_msg1.markers[i].color.g = 1.0;
          marker_msg1.markers[i].color.b = 0.0;

          marker_msg1.markers[i].points.push_back(detected);

          
          }

          artifact_marker_pub_.publish(marker_msg1);

        }


          // visualization_msgs::Marker marker_msg1;
          // marker_msg1.header.frame_id = "/world_shafter";
          // marker_msg1.header.stamp = ros::Time::now();
          // marker_msg1.type = visualization_msgs::Marker::SPHERE;
          // marker_msg1.action = visualization_msgs::Marker::ADD;
          // marker_msg1.mesh_use_embedded_materials = false;
          // marker_msg1.id = 0;
          // //-----place the coordinates of the artifact in this part to visualize
          // //----//
          // marker_msg1.pose.position.x = human_(0); // x-coordinate
          // marker_msg1.pose.position.y = human_(1); // y-coordinate
          // marker_msg1.pose.position.z = human_(2); // z-coordinate
          // //-----  ----//
          // marker_msg1.pose.orientation.x = 0;
          // marker_msg1.pose.orientation.y = 0;
          // marker_msg1.pose.orientation.z = 0.0;
          // marker_msg1.pose.orientation.w = 1.0;
          // marker_msg1.scale.x = 0.8;
          // marker_msg1.scale.y = 0.8;
          // marker_msg1.scale.z = 0.8;
          // marker_msg1.color.a = 1.0;
          // marker_msg1.color.r = 1.0;
          // marker_msg1.color.g = 0.0;
          // marker_msg1.color.b = 0.0;
          // artifact_marker_pub_.publish(marker_msg1);
          // marker_msg_array.markers.push_back(marker_msg1);
        // }
      // }
      // if (marker_msg_array.markers.size() != 0) {
      //  artifact_marker_pub_.publish(marker_msg_array);
      //}
    
  }


int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_node");
  tf::TransformListener listener;

  ros::NodeHandle nh;
  artifact_found_flag_pub_ =
      nh.advertise<std_msgs::String>("artifact_detected", 1);
  artifact_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "detected_artifact_visualization", 1);
  ros::Timer timer2 = nh.createTimer(ros::Duration(1.0), callback2);
  message_filters::Subscriber<Image> image1_sub(
      nh, "/camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> image2_sub(
      nh, "/darknet_ros/bounding_boxes", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odometry/imu",
                                                           1);
  tran = &listener;
  typedef sync_policies::ApproximateTime<Image, darknet_ros_msgs::BoundingBoxes,
                                         nav_msgs::Odometry>
      MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence
  // MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10000), image1_sub, image2_sub,
                                  odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}
