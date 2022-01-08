#pragma once           // Only include once per compile
#ifndef BEHAVIOR_EXECUTION  // Conditional compiling
#define BEHAVIOR_EXECUTION

// Includes
#include <ros/ros.h>  // ROS header
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <behavior_msgs/MoveToAction.h>



// Define class
class BehaviourExecution {
public:
  // Constructor and destructor
  BehaviourExecution(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~BehaviourExecution();

private:

  actionlib::SimpleActionServer<behavior_msgs::MoveToAction> as_moveto;

  // Private functions
  bool asMoveToStatus();
  bool as_moveto_continue;

  void asMoveToExecute(const behavior_msgs::MoveToGoalConstPtr& goal);
  // bool computePathAndMove(geometry_msgs::PoseStamped, ros::Publisher*, double = 1.0, double = 5.0);  

  // Private variables and objects
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // ros::Publisher local_pos_pub;
};

#endif