#pragma once           // Only include once per compile
#ifndef BEHAVIOR_EXECUTION  // Conditional compiling
#define BEHVAIOR_EXECUTION

// Includes
#include <ros/ros.h>  // ROS header
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/TransformStamped.h>

#include <behavior_msgs/EverythingElseAction.h>
#include <behavior_msgs/TakeOffAction.h>

// ################## Tank entrance include statements
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <inspectrone_library/roscpp_lib.h>


#include <unistd.h>
#include <termios.h>
// #include <conio.h>
#include <iostream>
#include <sys/ioctl.h>


// Define class
class InspectionExecution {
public:
  // Constructor and destructor
  // ##########################Enter tank public params
  InspectionExecution(ros::NodeHandle*, ros::NodeHandle, ros::Rate);
  ~InspectionExecution();
  void state_cb(const mavros_msgs::State::ConstPtr&);
  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
  void entrance_frontPoint_cb(const geometry_msgs::TransformStamped tf_w_entrance_frontPoint);
  
  void mainLoop();
  // void takeoff(geometry_msgs::PoseStamped);
  bool moveTo(geometry_msgs::PoseStamped, ros::Publisher*, double = 1.0, double = 5.0);
  char getch();
  bool kbhit();
  void holdPositionIfPaused();
  void lookForEntrance();
  void confirmEntrance();
  void askToRepeat();
  void correctPose();
  void waitToConfirmEntrance();
  void setCorrectionFlag();
  void updateEntranceFrontPoint();
  void calculateInitialEntrancePoint();
  // void setGoal(geometry_msgs::PoseStamped&, double, double);
  
  int stage;
  ros::Rate* rate;
  double rateHz = 20;
  int correctionNo = 0;
  int requiredCorrectionNo = 4;
  double fraction = 1.0;

  mavros_msgs::State current_state;
  geometry_msgs::PoseStamped current_pose, endPose, endPose2;
  geometry_msgs::PoseStamped pose_entrance_frontPoint;
  geometry_msgs::PoseStamped new_pose_entrance_frontPoint;
  
  std::vector<geometry_msgs::PoseStamped> path;
  bool calculatedPath = false;
  bool reached_pose2 = false;
  bool found_entrance_frontPoint = false;
  bool look_for_entrance = false;
  bool found_valid_entrance = false;
  bool toCorrectEntrance = false;
  bool found_possible_entrance = false;
  bool paused = false;
  bool outsideTank = true;
  bool tookOff = false;

  // Tank entrance public params end
  
private:

  // enter tank private params
  char kb_input = 0;

  geometry_msgs::PoseStamped takeoff_pose, pose2, lastStablePose;
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;
  ros::Time last_request, last_action;

  // ROS Publishers, Subscribers and Services
  ros::Subscriber state_sub;
  ros::Subscriber local_pos_sub;
  ros::Subscriber entranceFrontpoint_sub;
  ros::Publisher local_pos_pub;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  // tank entrance private params end

  actionlib::SimpleActionServer<behavior_msgs::EverythingElseAction> as_everythingelse;
  actionlib::SimpleActionServer<behavior_msgs::TakeOffAction> as_takeoff;

  // Private functions
  bool asEverythingElseStatus();
  bool asTakeOffStatus();
  bool as_everythingelse_continue;
  bool as_takeoff_continue;
  void asEverythingElseExecute(const behavior_msgs::EverythingElseGoalConstPtr& goal);
  void asTakeOffExecute(const behavior_msgs::TakeOffGoalConstPtr& goal);

  // Private variables and objects
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string exploration_planner_ns_;
  std::string global_planner_ns_;
  std::string local_planner_ns_;

};

#endif