#include "behavior_tree/inspection_behavior.hpp"

// Constructor
InspectionBehavior::InspectionBehavior(){};
InspectionBehavior::InspectionBehavior(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  nh_ = nh;  // Set nodehandle
  pnh_ = pnh;

  // Initialise variables / parameters to class variables
  std::string tree_xml_path;
  pnh_.getParam("tree_xml_path", tree_xml_path);
  if (!tree_xml_path.empty()) {
    tree_xml_path_ = tree_xml_path;
    ROS_INFO("Using tree loaded from file: %s", tree_xml_path_.c_str());
  } else {
    ROS_ERROR("Tree path not specified! Shutting down.");
  }
  std::string tree_log_path;
  pnh_.getParam("tree_log_directory", tree_log_path);
  if (!tree_log_path.empty()) {
    tree_log_path_ = tree_log_path;
    ROS_INFO("Logging to directory: %s", tree_log_path_.c_str());
  }
  pnh_.param("tree_tick_freq", tree_tick_freq_, 100.0);
  pnh_.param("live_monitoring", live_monitoring_, true);
  pnh_.param("live_monitoring_console", live_monitoring_console_, false);
  pnh_.param("logging", logging_, true);
  pnh_.param<std::string>("battery_topic", battery_topic, "/mavros/battery");

  // Set up the tree
  BT::BehaviorTreeFactory factory;
  // Registor nodes
  factory.registerSimpleCondition("Continue", std::bind(MAVInspectionNodes::CheckContinue));
  MAVInspectionNodes::setContinuePtr(&continue_bool);
  factory.registerSimpleCondition("IsBatteryOK", std::bind(MAVInspectionNodes::CheckBattery));
  MAVInspectionNodes::setBatteryPtr(&battery_msg);
  MAVInspectionNodes::setBatteryLimit(20.0); // Remaining capacity percentage
  factory.registerNodeType<MAVInspectionNodes::TakeOffAction>("TakeOff");
  factory.registerNodeType<MAVInspectionNodes::PublishLastStablePoseAction>("PublishLastStablePose");
  factory.registerNodeType<MAVInspectionNodes::LookForEntranceAction>("LookForEntrance");
  factory.registerNodeType<MAVInspectionNodes::WaitToConfirmEntranceAction>("WaitToConfirmEntrance");
  factory.registerNodeType<MAVInspectionNodes::GetEntranceFrontPointAndMoveAction>("GetEntranceFrontPointAndMove");
  factory.registerNodeType<MAVInspectionNodes::CorrectPoseAction>("CorrectPose");
  factory.registerNodeType<MAVInspectionNodes::EnterTankAction>("EnterTank");
  factory.registerNodeType<MAVInspectionNodes::LandAction>("Land");
  factory.registerNodeType<MAVInspectionNodes::DemoPathAction>("DemoPath");
  factory.registerNodeType<MAVInspectionNodes::MappingTemplateAction>("MappingTemplate");


  // Create the tree
  tree = factory.createTreeFromFile(tree_xml_path_);
  tree_status = BT::NodeStatus::RUNNING;
  BT::printTreeRecursively(tree.rootNode());
  
  if (live_monitoring_console_) {
    // This logger prints state changes on console
    tree_logger_cout_ptr = new BT::StdCoutLogger(tree);
  }
  if (logging_) {
    // Create log folder
    boost::filesystem::path log_path(tree_log_path_);
    try {
      boost::filesystem::create_directory(log_path);
    } catch (...) {
      ROS_WARN("Something went wrong with the logging directory, using default logging path (~/.ros)...");
      log_path = boost::filesystem::path("");
    }
    boost::filesystem::path tree_log_file = log_path / "bt_trace.fbl";
    ROS_INFO("Logging state changes to %s", tree_log_file.c_str());
    // This logger saves state changes on file
    tree_logger_file_ptr = new BT::FileLogger(tree, tree_log_file.c_str());
    boost::filesystem::path tree_trace_file = log_path / "bt_trace.json";
    ROS_INFO("Logging trace performance to %s", tree_trace_file.c_str());
    // This logger stores the execution time of each node
    tree_logger_minitrace_ptr = new BT::MinitraceLogger(tree, tree_trace_file.c_str());
  }
  if (live_monitoring_) {
    // This logger publish status changes using ZeroMQ. Used by Groot
    publisher_zmq_ptr = new BT::PublisherZMQ(tree);
  }

  // Initialise publishers and subscribers
  pub_state = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("state", 100);
  sub_battery = nh_.subscribe(battery_topic, 1, &InspectionBehavior::batteryCallback, this);
  // sub_temperature = nh_.subscribe(temperature_topic, 1, &InspectionBehavior::temperatureCallback, this);

  srv_continue_ = pnh_.advertiseService("continue", &InspectionBehavior::continueService, this);

  // TESTING //
  ros::Duration sleep_time = ros::Duration(10);
  sleep_time.sleep();
  ROS_INFO("STARTING BEHAVIOR...");
  // TESTING //

//   // set MoveTo Action arguments for pose
//   for( auto& node: tree.nodes )
// {
//     // Not a typo: it is "=", not "=="
//     if( auto action_moveto = <MAVInspectionNodes::MoveToAction*>( node.get() ))
//     {

//         action_moveto->init( moveto_goal_pose, fraction, sec);
//         // action_moveto->init( &InspectionBehavior::goal->moveto_goal_pose, &InspectionBehavior::goal->fraction, &InspectionBehavior::goal->sec);
//     }
// }

  // Create timer for ticking the tree
  tree_tick_timer = nh.createTimer(ros::Duration(1.0 / tree_tick_freq_), &InspectionBehavior::tickTree, this);

}

// Destructor
InspectionBehavior::~InspectionBehavior() {
  ROS_INFO("Destructing InspectionBehavior...");
  // Free up allocated memory
  delete publisher_zmq_ptr;
}

// Tick the tree
void InspectionBehavior::tickTree(const ros::TimerEvent& event) {
  if (tree_status == BT::NodeStatus::FAILURE) {
    ROS_WARN("Tree finished with status FAILURE, stopping tree ticking timer...");
    tree_tick_timer.stop();
    return;
  } 
  if (tree_status == BT::NodeStatus::SUCCESS) {
    ROS_WARN("Tree finished with status SUCCESS, stopping tree ticking timer...");
    tree_tick_timer.stop();
    return;
  }
  if (!ros::ok()) {
    ROS_WARN("ROS is down, stopping tree ticking timer...");
    tree_tick_timer.stop();
    return;
  }

  // ROS_INFO("Ticking tree...");

  tree_status = tree.tickRoot();
}

// void InspectionBehavior::pubState() {
//   // Publish current state

//   if(pub_state.getNumSubscribers() <= 0) {
//     return;
//   }

//   diagnostic_msgs::DiagnosticStatus msg;

//   if (tree_status == BT::NodeStatus::RUNNING) {
//     msg.level = diagnostic_msgs::DiagnosticStatus::OK;
//     msg.name = "mav_behavior";
//     msg.message = "Running";
//   } else if (tree_status == BT::NodeStatus::SUCCESS) {
//     msg.level = diagnostic_msgs::DiagnosticStatus::OK;
//     msg.name = "mav_behavior";
//     msg.message = "Success";
//   } else if (tree_status == BT::NodeStatus::IDLE) {
//     msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
//     msg.name = "mav_behavior";
//     msg.message = "Idle";
//   } else { // tree_status == BT::NodeStatus::FAILURE
//     msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
//     msg.name = "mav_behavior";
//     msg.message = "Failure";
//   }
  
//   pub_state.publish(msg);
// }

void InspectionBehavior::batteryCallback(sensor_msgs::BatteryState msg) {
  // Get current battery status

  battery_msg = msg;
}

// void InspectionBehavior::temperatureCallback(sensor_msgs::Temperature msg) {
//   // Get current temperature status

//   temperature_msg = msg;
// }

bool InspectionBehavior::continueService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  res.success = true;

  continue_bool = req.data;
  ROS_INFO("Behaviour continue toggled to: %s", (continue_bool ? "True" : "False"));

  return res.success;
}

// void InspectionBehavior::setGoal(geometry_msgs::PoseStamped& newPose, double fraction, double sec )
// {
//     moveto_goal_pose = newPose;
//     this->fraction= fraction;
//     this->sec = sec;
// }

