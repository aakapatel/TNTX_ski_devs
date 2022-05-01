#include <ros/ros.h>
#include "visualization_tools/visualize_navigation3.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_navigation");
  ros::NodeHandle nodeHandle("~");

  visualization_tools::VisualizeNavigation3 VisualizeNavigation3(nodeHandle);

  ros::spin();
  return 0;
}
