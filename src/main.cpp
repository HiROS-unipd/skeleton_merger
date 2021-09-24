// ROS dependencies
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_merger/Merger.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hiros_skeleton_merger");

  hiros::merge::Merger sm;
  sm.start();

  exit(EXIT_SUCCESS);
}
