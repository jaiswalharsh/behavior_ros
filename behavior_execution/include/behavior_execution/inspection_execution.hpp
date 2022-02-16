#pragma once           // Only include once per compile
#ifndef BEHAVIOR_EXECUTION  // Conditional compiling
#define BEHVAIOR_EXECUTION

// Includes
#include <ros/ros.h>  // ROS header
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/TransformStamped.h>

#include <behavior_msgs/PublishLastStablePoseAction.h>
#include <behavior_msgs/TakeOffAction.h>
#include <behavior_msgs/LookForEntranceAction.h>
#include <behavior_msgs/WaitToConfirmEntranceAction.h>
#include <behavior_msgs/GetEntranceFrontPointAndMoveAction.h>
#include <behavior_msgs/CorrectPoseAction.h>
#include <behavior_msgs/EnterTankAction.h>
#include <behavior_msgs/LandAction.h>
#include <behavior_msgs/DemoPathAction.h>
#include <behavior_msgs/MappingTemplateAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <inspectrone_library/roscpp_lib.h>


#include <unistd.h>
// #include <conio.h>
#include <iostream>

// Define class
class InspectionExecution {
public:
  // Constructor and destructor
  InspectionExecution(ros::NodeHandle* nh, ros::NodeHandle& pnh, ros::Rate);
  ~InspectionExecution();
  void state_cb(const mavros_msgs::State::ConstPtr&);
  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
  void entrance_frontPoint_cb(const geometry_msgs::TransformStamped tf_w_entrance_frontPoint);
  // bool moveTo(geometry_msgs::PoseStamped, ros::Publisher*, double = 1.0, double = 5.0);
  bool moveTo(geometry_msgs::PoseStamped, ros::Publisher*, double = 1.0, double = 5.0, std::string = "default");
  void holdPositionIfPaused();
  // void askToRepeat();
  void correctPose();
  void setCorrectionFlag();
  void updateEntranceFrontPoint();
  bool isPositionReached(geometry_msgs::PoseStamped);
  void updateRobotState();
  // void setGoal(geometry_msgs::PoseStamped&, double, double);
  
  int stage;
  ros::Rate* rate;
  double rateHz = 20;
  int correctionNo = 0;
  int requiredCorrectionNo;
  double fraction = 1.0;
  double position_tolerance_;
  double moveSpeed;

  mavros_msgs::State current_state;
  geometry_msgs::PoseStamped current_pose, endPose, endPose2, endPose3, landPose;
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
  bool inMovement = false;

  
private:

  char kb_input = 0;
  std_msgs::UInt8 led_on_value;
  std_msgs::UInt8 led_off_value;

  geometry_msgs::PoseStamped takeoff_pose, pose2, lastStablePose;
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;
  mavros_msgs::CommandBool disarm_cmd;
  ros::Time last_request, last_action;

  // ROS Publishers, Subscribers and Services
  ros::Subscriber state_sub;
  ros::Subscriber local_pos_sub;
  ros::Subscriber entranceFrontpoint_sub;
  ros::Publisher local_pos_pub;
  ros::Publisher led_pub;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

  actionlib::SimpleActionServer<behavior_msgs::PublishLastStablePoseAction> as_publishlaststablepose;
  actionlib::SimpleActionServer<behavior_msgs::TakeOffAction> as_takeoff;
  actionlib::SimpleActionServer<behavior_msgs::LookForEntranceAction> as_lookforentrance;
  actionlib::SimpleActionServer<behavior_msgs::WaitToConfirmEntranceAction> as_waittoconfirmentrance;
  actionlib::SimpleActionServer<behavior_msgs::GetEntranceFrontPointAndMoveAction> as_getentrancefrontpointandmove;
  actionlib::SimpleActionServer<behavior_msgs::CorrectPoseAction> as_correctpose;
  actionlib::SimpleActionServer<behavior_msgs::EnterTankAction> as_entertank;
  actionlib::SimpleActionServer<behavior_msgs::LandAction> as_land;
  actionlib::SimpleActionServer<behavior_msgs::DemoPathAction> as_demopath;
  actionlib::SimpleActionServer<behavior_msgs::MappingTemplateAction> as_mappingtemplate;

  // Action methods
  bool asPublishLastStablePoseStatus();
  bool asTakeOffStatus();
  bool asLookForEntranceStatus();
  bool asWaitToConfirmEntranceStatus();
  bool asGetEntranceFrontPointAndMoveStatus();
  bool asCorrectPoseStatus();
  bool asEnterTankStatus();
  bool asLandStatus();
  bool asDemoPathStatus();
  bool asMappingTemplateStatus();
  bool as_publishlaststablepose_continue;
  bool as_takeoff_continue;
  bool as_lookforentrance_continue;
  void asPublishLastStablePoseExecute(const behavior_msgs::PublishLastStablePoseGoalConstPtr& goal);
  void asTakeOffExecute(const behavior_msgs::TakeOffGoalConstPtr& goal);
  void asLookForEntranceExecute(const behavior_msgs::LookForEntranceGoalConstPtr& goal);
  void asWaitToConfirmEntranceExecute(const behavior_msgs::WaitToConfirmEntranceGoalConstPtr& goal);
  void asGetEntranceFrontPointAndMoveExecute(const behavior_msgs::GetEntranceFrontPointAndMoveGoalConstPtr& goal);
  void asCorrectPoseExecute(const behavior_msgs::CorrectPoseGoalConstPtr& goal);
  void asEnterTankExecute(const behavior_msgs::EnterTankGoalConstPtr& goal);
  void asLandExecute(const behavior_msgs::LandGoalConstPtr& goal);
  void asDemoPathExecute(const behavior_msgs::DemoPathGoalConstPtr& goal);
  void asMappingTemplateExecute(const behavior_msgs::MappingTemplateGoalConstPtr& goal);



  // Private variables and objects
  ros::NodeHandle *nh_;
  ros::NodeHandle pnh_;

  // std::string exploration_planner_ns_;
  // std::string global_planner_ns_;
  // std::string local_planner_ns_;

};

#endif