#include <ros/ros.h>

#include "@project_name@/@project_name@.h"

int main(int argc, char *argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "@project_name@");

  ROS_INFO("Hello World");

  return EXIT_SUCCESS;
}
