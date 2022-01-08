#include "behavior_execution/inspection_execution.hpp"

int main(int argc, char** argv) {
  // Create node
  ros::init(argc, argv, "behavior_execution");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");  // Private nodehandle for parameters
  ros::Rate rate(20);

  // Create object
  ROS_INFO("Setting up class");
  InspectionExecution node(&nh, pnh, rate);


  ROS_INFO("Spinning...");
  ros::Rate r(5.0); // Defining the looping rate in Hz
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}