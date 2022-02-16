#pragma once           // Only include once per compile
#ifndef BEHAVIOR_TREE  // Conditional compiling
#define BEHAVIOR_TREE

// Includes
#include <boost/filesystem.hpp>
#include <ros/ros.h>  // ROS header

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/basic_types.h>

#include "behavior_tree/inspection_nodes.hpp"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/SetBool.h>


// Define class
class InspectionBehavior {
public:
  // Constructor and destructor
  InspectionBehavior(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~InspectionBehavior();

  // Public functions
  void tickTree(const ros::TimerEvent& event);

  // Public variables and objects
  // struct MoveToParams;    
  // struct MoveToParams
  // {
  //     geometry_msgs::PoseStamped moveto_goal_pose;
  //     double fraction = 1.0;
  //     double sec = 5.0;
  // }params;
  
private:
  // Private functions
  void pubState();
  void batteryCallback(sensor_msgs::BatteryState msg);
  // void temperatureCallback(sensor_msgs::Temperature msg);
  bool continueService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  


  // Private variables and objects
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher pub_state;
  ros::Subscriber sub_battery;
  // ros::Subscriber sub_temperature;
  ros::ServiceServer srv_continue_;

  // struct Position3D 
  // { 
  //   double x;
  //   double y;
  //   double z; 
  // };

  // Parameters
  std::string battery_topic;
  // std::string temperature_topic;
  std::string tree_xml_path_;
  std::string tree_log_path_;
  bool live_monitoring_;
  bool live_monitoring_console_;
  bool logging_;
  double tree_tick_freq_;

  ros::Timer tree_tick_timer;
  BT::Tree tree;
  BT::NodeStatus tree_status;
  BT::StdCoutLogger* tree_logger_cout_ptr;
  BT::FileLogger* tree_logger_file_ptr;
  BT::MinitraceLogger* tree_logger_minitrace_ptr;
  BT::PublisherZMQ* publisher_zmq_ptr;

  bool continue_bool = true;
  sensor_msgs::BatteryState battery_msg;
  // sensor_msgs::Temperature temperature_msg;

};

#endif