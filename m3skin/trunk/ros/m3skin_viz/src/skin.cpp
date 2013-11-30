#include <ros/ros.h>

#include "RosPressureVisualizer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m3skin_viz");

  m3::RosPressureVisualizer node;

  node.Init();

  ros::spin();

  return 0;
}
