#include <ros/ros.h>
#include "visualization_tools/visualize_navigation.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_navigation");
  ros::NodeHandle nodeHandle("~");

  visualization_tools::VisualizeNavigation VisualizeNavigation(nodeHandle);

  ros::spin();
  return 0;
}
