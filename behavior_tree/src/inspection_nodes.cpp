#include "behavior_tree/inspection_nodes.hpp"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory) {
  factory.registerSimpleCondition("Continue", std::bind(MAVInspectionNodes::CheckContinue));
  factory.registerSimpleCondition("IsBatteryOK", std::bind(MAVInspectionNodes::CheckBattery));
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
    battery_level = batt_ptr->percentage * 100;
    
    if (battery_level >= batt_limit) {
      battery_level_ok = true;
      // ROS_INFO("battery level is okay");
    }else {
      battery_level_ok = true;
      ROS_INFO_THROTTLE(5,"battery level is not okay: %3.2f", battery_level);
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
      ROS_INFO_THROTTLE(5, "[Battery] SOC: %3.2f", battery_level);
      status = BT::NodeStatus::SUCCESS;
      } else {
      ROS_WARN_THROTTLE(5, "[Battery] SOC: %3.2f", battery_level);
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

    // std::string tmp = execution_ns_ +"/takeoff" + "_1";
    // ROS_INFO("%s\n", tmp.c_str());
    // Start action client
    if (!ac_.waitForServer(ros::Duration(5.0))) { // 5 seconds
      ROS_WARN("[TakeOff: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
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
    // Wait in the while loop for the action client to return success
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

  BT::NodeStatus PublishLastStablePoseAction::tick() {
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
    behavior_msgs::PublishLastStablePoseGoal goal;
    // Send goal
    ROS_INFO("[PublishLastStablePose: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&PublishLastStablePoseAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::PublishLastStablePoseAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::PublishLastStablePoseAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[PublishLastStablePose: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[PublishLastStablePose: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[PublishLastStablePose: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[PublishLastStablePose: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[PublishLastStablePose] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[PublishLastStablePose: FINISHED]");
    return status;
  }

  void PublishLastStablePoseAction::halt() {
    _halt_requested.store(true);
  }

  void PublishLastStablePoseAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::PublishLastStablePoseResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }


  //////////////////////////////////////// Look For Entrance action ////////////////////////////////////////

  BT::NodeStatus LookForEntranceAction::tick() {
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

    // std::string tmp = execution_ns_ +"/lookforentrance" + "_1";
    // ROS_INFO("%s\n", tmp.c_str());
    // Start action client
    if (!ac_.waitForServer(ros::Duration(5.0))) { // 5 seconds
      ROS_WARN("[LookForEntrance: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[LookForEntrance: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::LookForEntranceGoal goal;
    // Send goal
    ROS_INFO("[LookForEntrance: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&LookForEntranceAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::LookForEntranceAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::LookForEntranceAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[LookForEntrance: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[LookForEntrance: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[LookForEntrance: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[LookForEntrance: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[LookForEntrance] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[LookForEntrance: FINISHED]");
    return status;
  }

  void LookForEntranceAction::halt() {
    _halt_requested.store(true);
  }

  void LookForEntranceAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::LookForEntranceResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }

  //////////////////////////////////////// Wait To Confirm Entrance action ////////////////////////////////////////

  BT::NodeStatus WaitToConfirmEntranceAction::tick() {
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
      ROS_WARN("[WaitToConfirmEntrance: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[WaitToConfirmEntrance: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::WaitToConfirmEntranceGoal goal;
    // Send goal
    ROS_INFO("[WaitToConfirmEntrance: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&WaitToConfirmEntranceAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::WaitToConfirmEntranceAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::WaitToConfirmEntranceAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[WaitToConfirmEntrance: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[WaitToConfirmEntrance: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[WaitToConfirmEntrance: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[WaitToConfirmEntrance: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[WaitToConfirmEntrance] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[WaitToConfirmEntrance: FINISHED]");
    return status;
  }

  void WaitToConfirmEntranceAction::halt() {
    _halt_requested.store(true);
  }

  void WaitToConfirmEntranceAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::WaitToConfirmEntranceResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }

  //////////////////////////////////////// Calculate Initial Entrance Point action ////////////////////////////////////////

  BT::NodeStatus GetEntranceFrontPointAndMoveAction::tick() {
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
      ROS_WARN("[GetEntranceFrontPointAndMove: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[GetEntranceFrontPointAndMove: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::GetEntranceFrontPointAndMoveGoal goal;
    // Send goal
    ROS_INFO("[GetEntranceFrontPointAndMove: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&GetEntranceFrontPointAndMoveAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::GetEntranceFrontPointAndMoveAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::GetEntranceFrontPointAndMoveAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[GetEntranceFrontPointAndMove: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[GetEntranceFrontPointAndMove: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[GetEntranceFrontPointAndMove: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[GetEntranceFrontPointAndMove: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[GetEntranceFrontPointAndMove] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[GetEntranceFrontPointAndMove: FINISHED]");
    return status;
  }

  void GetEntranceFrontPointAndMoveAction::halt() {
    _halt_requested.store(true);
  }

  void GetEntranceFrontPointAndMoveAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::GetEntranceFrontPointAndMoveResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }



  //////////////////////////////////////// Correct Pose action ////////////////////////////////////////

  BT::NodeStatus CorrectPoseAction::tick() {
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
      ROS_WARN("[CorrectPose: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[CorrectPose: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::CorrectPoseGoal goal;
    // Send goal
    ROS_INFO("[CorrectPose: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&CorrectPoseAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::CorrectPoseAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::CorrectPoseAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[CorrectPose: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[CorrectPose: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[CorrectPose: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[CorrectPose: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[CorrectPose] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[CorrectPose: FINISHED]");
    return status;
  }

  void CorrectPoseAction::halt() {
    _halt_requested.store(true);
  }

  void CorrectPoseAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::CorrectPoseResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }



  //////////////////////////////////////// Enter Tank action ////////////////////////////////////////

  BT::NodeStatus EnterTankAction::tick() {
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
      ROS_WARN("[EnterTank: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[EnterTank: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::EnterTankGoal goal;
    // Send goal
    ROS_INFO("[EnterTank: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&EnterTankAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::EnterTankAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::EnterTankAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[EnterTank: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[EnterTank: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[EnterTank: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[EnterTank: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[EnterTank] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[EnterTank: FINISHED]");
    return status;
  }

  void EnterTankAction::halt() {
    _halt_requested.store(true);
  }

  void EnterTankAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::EnterTankResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }



    //////////////////////////////////////// Land action ////////////////////////////////////////

  BT::NodeStatus LandAction::tick() {
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
      ROS_WARN("[Land: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[Land: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::LandGoal goal;
    // Send goal
    ROS_INFO("[Land: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&LandAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::LandAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::LandAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[Land: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[Land: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[Land: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[Land: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[Land] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[Land: FINISHED]");
    return status;
  }

  void LandAction::halt() {
    _halt_requested.store(true);
  }

  void LandAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::LandResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }

      //////////////////////////////////////// Demo Path action ////////////////////////////////////////

  BT::NodeStatus DemoPathAction::tick() {
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
      ROS_WARN("[DemoPath: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[DemoPath: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::DemoPathGoal goal;
    // Send goal
    ROS_INFO("[DemoPath: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&DemoPathAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::DemoPathAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::DemoPathAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[DemoPath: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[DemoPath: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[DemoPath: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[DemoPath: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[DemoPath] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[DemoPath: FINISHED]");
    return status;
  }

  void DemoPathAction::halt() {
    _halt_requested.store(true);
  }

  void DemoPathAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::DemoPathResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }



        //////////////////////////////////////// Mapping Template action ////////////////////////////////////////

  BT::NodeStatus MappingTemplateAction::tick() {
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
      ROS_WARN("[MappingTemplate: FAILED TO START] ActionClient could not connect to ActionServer (with connection timeout = 5 seconds)");
      return BT::NodeStatus::FAILURE;
    }
    // ac_.waitForServer();
    // Print debug
    ROS_INFO("[MappingTemplate: STARTED] Timeout: %f", timeout);

    // Initialise execution
    _halt_requested.store(false);
    ros::Time time_started = ros::Time::now();
    ros::Duration time_timeout = ros::Duration(timeout);
    ros::Time time_max = time_started + time_timeout;
    ros::Rate r = ros::Rate(10.0); // 10 Hz

    // Send the goal using a ROS action
    // Make goal
    behavior_msgs::MappingTemplateGoal goal;
    // Send goal
    ROS_INFO("[MappingTemplate: RUNNING] Sending action goal...");
    ac_done = false;
    ac_success = false;
    ac_.sendGoal(goal,
                boost::bind(&MappingTemplateAction::doneCB, this, _1, _2),
                actionlib::SimpleActionClient<behavior_msgs::MappingTemplateAction>::SimpleActiveCallback(),    // Do nothing
                actionlib::SimpleActionClient<behavior_msgs::MappingTemplateAction>::SimpleFeedbackCallback()); // Do nothing

    // Perform action
    // Wait in the while loop for the action client to return success
    while (true) {
      // Check for halt
      if (_halt_requested) { 
        status = BT::NodeStatus::FAILURE;
        ROS_INFO("[MappingTemplate: FAILED] Halt requested");
        break;
      } 

      // Check for timeout
      if (check_timeout) {
        ros::Time time_now = ros::Time::now();
        if (time_now >= time_max) { // Check for timeout
          status = BT::NodeStatus::FAILURE;
          ROS_INFO("[MappingTemplate: FAILED] Timeout exceeded");
          break;
        } 
      }

      // Check for action client done
      if (ac_done) {
        if (ac_success) {
          ROS_INFO("[MappingTemplate: SUCCESS] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::SUCCESS;
        } else {
          ROS_WARN("[MappingTemplate: FAILURE] Action finished with state %s", ac_.getState().toString().c_str());
          status = BT::NodeStatus::FAILURE;
        }
        break;
      }

      // Sleep
      r.sleep();
    }

    // Handle loop exit before action client is done
    if (status == BT::NodeStatus::FAILURE && !ac_done) {
      ROS_WARN("[MappingTemplate] Cancelling action goal...");
      // Cancel current goal
      ac_.cancelGoal();
    }

    ROS_INFO("[MappingTemplate: FINISHED]");
    return status;
  }

  void MappingTemplateAction::halt() {
    _halt_requested.store(true);
  }

  void MappingTemplateAction::doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::MappingTemplateResultConstPtr& result) {
    ac_done = state.isDone();
    ac_success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }

} // end namespace MAVInspectionNodes
