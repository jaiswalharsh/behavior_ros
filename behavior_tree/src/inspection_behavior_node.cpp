#include "behavior_tree/inspection_behavior.hpp"

int main(int argc, char** argv) {
  // Create node
  ros::init(argc, argv, "behavior_tree");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");  // Private nodehandle for parameters

  // Create object
  ROS_INFO("Setting up class");
  InspectionBehavior node(nh, pnh);

  /*                    Simple spinning                           */
  ros::spin();


  /*                    Periodic spinning with rate               */
  /*  ROS_INFO("Spinning...");
   *  ros::Rate rate(1.0); // Defing the looping rate
   *  while (ros::ok())
   *  {
   *      // node.doSomething();
   *      ros::spinOnce();
   *      rate.sleep();
   *  }
   */

  /*                    Asynchronous / multithreaded spinning     */
  /*  // Start spinner
   *  ros::AsyncSpinner spinner(1); // 1 thread
   *  spinner.start();
   *  // Spin until shutdown
   *  ros::waitForShutdown();
   */

  return 0;
}