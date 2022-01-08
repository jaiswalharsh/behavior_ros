#include "behavior_tree/inspection_nodes.hpp"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory) {
  factory.registerSimpleCondition("Continue", std::bind(MAVInspectionNodes::CheckContinue));
  factory.registerSimpleCondition("IsBatteryOK", std::bind(MAVInspectionNodes::CheckBattery));
  factory.registerNodeType<MAVInspectionNodes::TakeOffAction>("TakeOff");
  factory.registerNodeType<MAVInspectionNodes::EverythingElseAction>("EverythingElse");
}


namespace MAVInspectionNodes
{

void setBatteryPtr(sensor_msgs::BatteryState *battery_msg_ptr) {
  batt_ptr = battery_msg_ptr;
}
void setBatteryLimit(double limit) {
  batt_limit = limit;
}

void setContinuePtr(bool *continue_ptr) {
  cont_ptr = continue_ptr;
}

//////////////////////////////////////// Continue condition ////////////////////////////////////////

// Return SUCCESS if exploration is finished, i.e. all rooms have been explored, otherwise FAILURE
BT::NodeStatus CheckContinue()
{
  // Default to FAILURE
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  
  if (*cont_ptr == true) {
    status = BT::NodeStatus::SUCCESS;
  } else {
    status = BT::NodeStatus::FAILURE;
  }

  // // TESTING //
  // ros::Duration sleep_time = ros::Duration(0.5);
  // sleep_time.sleep();
  // status = BT::NodeStatus::SUCCESS;
  status = BT::NodeStatus::SUCCESS;
  // // TESTING //
  
  return status;
}



//////////////////////////////////////// Battery condition ////////////////////////////////////////

// Return SUCCESS for battery OK, or FAILURE for critical battery level
BT::NodeStatus CheckBattery()
{
  // Default to FAILURE
  BT::NodeStatus status = BT::NodeStatus::FAILURE;

  // Initialise values
  double battery_level;
  int battery_health;
  bool battery_health_ok, battery_level_ok, battery_ok;
  // Get values from message
  battery_level = batt_ptr->percentage;
  if (battery_level >= batt_limit) {
    battery_level_ok = true;
  }else {
    battery_level_ok = false;
  }
  // battery_health = batt_ptr->power_supply_health;
  // if (battery_health == sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD) {
  //   battery_health_ok = true;
  // } else {
  //   battery_health_ok = false;
  // }
  // Compute status
  //battery_ok = (battery_level_ok && battery_health_ok);
  // battery_ok = battery_level_ok;

  // Report battery status
  // Commented as battery health status info is unknown
  // if (battery_level_ok) {
  //   ROS_INFO_THROTTLE(5, "[Battery] SOC: %3.2f, Status: %d", battery_level * 100, battery_health);
  //   status = BT::NodeStatus::SUCCESS;
  // } else {
  //   ROS_WARN_THROTTLE(5, "[Battery] SOC: %3.2f, Status: %d", battery_level * 100, battery_health);
  //   status = BT::NodeStatus::FAILURE;
  // }

  if (battery_level_ok) {
    ROS_INFO_THROTTLE(5, "[Battery] SOC: %3.2f", battery_level * 100);
    status = BT::NodeStatus::SUCCESS;
    } else {
    ROS_WARN_THROTTLE(5, "[Battery] SOC: %3.2f", battery_level * 100);
    status = BT::NodeStatus::FAILURE;
  }

  // // TESTING //
  // ros::Duration sleep_time = ros::Duration(0.5);
  // sleep_time.sleep();
  // static int counter = 0;
  // if (++counter <= 15) {
  //   status = BT::NodeStatus::SUCCESS;
  // } else {
  //   status = BT::NodeStatus::FAILURE;
  // }
  // status = BT::NodeStatus::SUCCESS;
  // // TESTING //

  
  
  return status;
}

//////////////////////////////////////// Take Off action ////////////////////////////////////////

BT::NodeStatus TakeOffAction::tick() {
  // Initialise status
  BT::NodeStatus status = BT::NodeStatus::FAILURE;

  // Get ports
  double timeout = 0.0;
  // if (!getInput<double>("timeout", timeout)) {
  //   throw BT::RuntimeError("Missing required input [timeout]");
  // }
  bool check_timeout = true;
  if (timeout == 0.0) {
    check_timeout = false;
  }

  // Start action client
  if (!ac_.waitForServer(ros::Duration(5.0))) { // 5 seconds
    ROS_WARN("[TakeOff: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
    return BT::NodeStatus::FAILURE;
  }

  // Print debug
  ROS_INFO("[TakeOff: STARTED] Timeout: %f", timeout);

  // Initialise execution
  _halt_requested.store(false);
  ros::Time time_started = ros::Time::now();
  ros::Duration time_timeout = ros::Duration(timeout);
  ros::Time time_max = time_started + time_timeout;
  ros::Rate r = ros::Rate(10.0); // 10 Hz

  // Send the goal using a ROS action
  // Make goal
  behavior_msgs::TakeOffGoal goal;
  // Send goal
  ROS_INFO("[TakeOff: RUNNING] Sending action goal...");
  ac_done = false;
  ac_success = false;
  ac_.sendGoal(goal,
               boost::bind(&TakeOffAction::doneCB, this, _1, _2),
               actionlib::SimpleActionClient<behavior_msgs::TakeOffAction>::SimpleActiveCallback(),    // Do nothing
               actionlib::SimpleActionClient<behavior_msgs::TakeOffAction>::SimpleFeedbackCallback()); // Do nothing

  // Perform action
  while (true) {
    // Check for halt
    if (_halt_requested) { 
      status = BT::NodeStatus::FAILURE;
      ROS_INFO("[TakeOff: FAILED] Halt requested");
      break;
    } 

    // Check for timeout
    if (check_timeout) {
      ros::Time time_now = ros::Time::now();
      if (time_now >= time_max) { // Check for timeout
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[TakeOff: FAILED] Timeout exceeded");
        break;
      } 
    }

    // Check for action client done
    if (ac_done) {
      if (ac_success) {
        ROS_INFO("[TakeOff: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
        status = BT::NodeStatus::SUCCESS;
      } else {
        ROS_WARN("[TakeOff: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
        status = BT::NodeStatus::FAILURE;
      }
      break;
    }

    // Sleep
    r.sleep();
  }

  // Handle loop exit before action client is done
  if (status == BT::NodeStatus::FAILURE && !ac_done) {
    ROS_WARN("[TakeOff] Cancelling action goal...");
    // Cancel current goal
    ac_.cancelGoal();
  }

  ROS_INFO("[TakeOff: FINISHED]");
  return status;
}

void TakeOffAction::halt() {
  _halt_requested.store(true);
}

void TakeOffAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::TakeOffResultConstPtr& result) {
  ac_done = state.isDone();
  ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
}




//////////////////////////////////////// Everything Else action ////////////////////////////////////////

BT::NodeStatus EverythingElseAction::tick() {
  // Initialise status
  BT::NodeStatus status = BT::NodeStatus::SUCCESS;

  // Get ports
  double timeout = 0.0;
  // if (!getInput<double>("timeout", timeout)) {
  //   throw BT::RuntimeError("Missing required input [timeout]");
  // }
  bool check_timeout = true;
  if (timeout == 0.0) {
    check_timeout = false;
  }

  // Start action client
  if (!ac_.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("[Everything Else: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
    return BT::NodeStatus::FAILURE;
  }

  // Print debug
  ROS_INFO("[Everything Else: STARTED] Timeout: %f", timeout);

  // Initialise execution
  _halt_requested.store(false);
  ros::Time time_started = ros::Time::now();
  ros::Duration time_timeout = ros::Duration(timeout);
  ros::Time time_max = time_started + time_timeout;
  ros::Rate r = ros::Rate(10.0);

  // Send the goal using a ROS action
  // Make goal
  behavior_msgs::EverythingElseGoal goal;
  // Send goal
  ROS_INFO("[EverythingElse: RUNNING] Sending action goal...");
  ac_done = false;
  ac_success = false;
  ac_.sendGoal(goal,
               boost::bind(&EverythingElseAction::doneCB, this, _1, _2),
               actionlib::SimpleActionClient<behavior_msgs::EverythingElseAction>::SimpleActiveCallback(),    // Do nothing
               actionlib::SimpleActionClient<behavior_msgs::EverythingElseAction>::SimpleFeedbackCallback()); // Do nothing

  // Perform action
  while (true) {
    // Check for halt
    if (_halt_requested) { 
      status = BT::NodeStatus::FAILURE;
      ROS_INFO("[EverythingElse: FAILED] Halt requested");
      break;
    } 

    // Check for timeout
    if (check_timeout) {
      ros::Time time_now = ros::Time::now();
      if (time_now >= time_max) { // Check for timeout
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[EverythingElse: FAILED] Timeout exceeded");
        break;
      } 
    }

    // Check for action client done
    if (ac_done) {
      if (ac_success) {
        ROS_INFO("[EverythingElse: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
        status = BT::NodeStatus::SUCCESS;
      } else {
        ROS_WARN("[EverythingElse: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
        status = BT::NodeStatus::FAILURE;
      }
      break;
    }

    // Sleep
    r.sleep();
  }

  // Handle loop exit before action client is done
  if (status == BT::NodeStatus::FAILURE && !ac_done) {
    ROS_WARN("[EverythingElse] Cancelling action goal...");
    // Cancel current goal
    ac_.cancelGoal();
  }

  ROS_INFO("[EverythingElse: FINISHED]");
  return status;
}

void EverythingElseAction::halt() {
  _halt_requested.store(true);
}

void EverythingElseAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::EverythingElseResultConstPtr& result) {
  ac_done = state.isDone();
  ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
}


//////////////////////////////////////// Move To action ////////////////////////////////////////

// BT::NodeStatus MoveToAction::tick() {
//   // Initialise status
//   BT::NodeStatus status = BT::NodeStatus::FAILURE;

//   // Get ports
//   double timeout;
//   if (!getInput<double>("timeout", timeout)) {
//     throw BT::RuntimeError("Missing required input [timeout]");
//   }
//   bool check_timeout = true;
//   if (timeout == 0.0) {
//     check_timeout = false;
//   }

//   // Start action client
//   if (!ac_.waitForServer(ros::Duration(5.0))) { // 5 seconds
//     ROS_WARN("[MoveTo: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
//     return BT::NodeStatus::FAILURE;
//   }

//   // Print debug
//   ROS_INFO("[MoveTo: STARTED] Timeout: %f", timeout);

//   // Initialise execution
//   _halt_requested.store(false);
//   ros::Time time_started = ros::Time::now();
//   ros::Duration time_timeout = ros::Duration(timeout);
//   ros::Time time_max = time_started + time_timeout;
//   ros::Rate r = ros::Rate(10.0); // 10 Hz

//   // Send the goal using a ROS action
//   // Make goal
//   behavior_msgs::MoveToGoal goal;
//   goal.goal_pose = _moveto_goal_pose;
//   goal.fraction = _fraction;
//   goal.sec = _sec;
//   // Send goal
//   ROS_INFO("[MoveTo: RUNNING] Sending action goal...");
//   ac_done = false;
//   ac_success = false;
//   ac_.sendGoal(goal,
//                boost::bind(&MoveToAction::doneCB, this, _1, _2),
//                actionlib::SimpleActionClient<behavior_msgs::MoveToAction>::SimpleActiveCallback(),    // Do nothing
//                actionlib::SimpleActionClient<behavior_msgs::MoveToAction>::SimpleFeedbackCallback()); // Do nothing

//   // Perform action
//   while (true) {
//     // Check for halt
//     if (_halt_requested) { 
//       status = BT::NodeStatus::FAILURE;
//       ROS_INFO("[MoveTo: FAILED] Halt requested");
//       break;
//     } 

//     // Check for timeout
//     if (check_timeout) {
//       ros::Time time_now = ros::Time::now();
//       if (time_now >= time_max) { // Check for timeout
//         status = BT::NodeStatus::FAILURE;
//         ROS_INFO("[MoveTo: FAILED] Timeout exceeded");
//         break;
//       } 
//     }

//     // Check for action client done
//     if (ac_done) {
//       if (ac_success) {
//         ROS_INFO("[MoveTo: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
//         status = BT::NodeStatus::SUCCESS;
//       } else {
//         ROS_WARN("[MoveTo: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
//         status = BT::NodeStatus::FAILURE;
//       }
//       break;
//     }

//     // Sleep
//     r.sleep();
//   }

//   // Handle loop exit before action client is done
//   if (status == BT::NodeStatus::FAILURE && !ac_done) {
//     ROS_WARN("[MoveTo] Cancelling action goal...");
//     // Cancel current goal
//     ac_.cancelGoal();
//   }

//   ROS_INFO("[MoveTo: FINISHED]");
//   return status;
// }

// void MoveToAction::halt() {
//   _halt_requested.store(true);
// }

// void MoveToAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::MoveToResultConstPtr& result) {
//   ac_done = state.isDone();
//   ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
// }
} // end namespace MAVInspectionNodes
