#include "behavior_execution/inspection_execution.hpp"


// Destructor
InspectionExecution::~InspectionExecution() {
  ROS_INFO("Destructing InspectionExecution...");
  // Free up allocated memory
  // delete tf_listener_ptr_;
  // delete tf_broadcaster_ptr_;
}

// Constructor
InspectionExecution::InspectionExecution(ros::NodeHandle* nh, ros::NodeHandle& pnh, ros::Rate rate1) : 
  as_publishlaststablepose(pnh, "publishlaststablepose", boost::bind(&InspectionExecution::asPublishLastStablePoseExecute, this, _1), true), 
  as_takeoff(pnh,"takeoff", boost::bind(&InspectionExecution::asTakeOffExecute, this, _1), true),
  as_lookforentrance(pnh,"lookforentrance", boost::bind(&InspectionExecution::asLookForEntranceExecute, this, _1), true),
  as_waittoconfirmentrance(pnh,"waittoconfirmentrance", boost::bind(&InspectionExecution::asWaitToConfirmEntranceExecute, this, _1), true),
  as_getentrancefrontpointandmove(pnh,"getentrancefrontpointandmove", boost::bind(&InspectionExecution::asGetEntranceFrontPointAndMoveExecute, this, _1), true),
  as_correctpose(pnh,"correctpose", boost::bind(&InspectionExecution::asCorrectPoseExecute, this, _1), true),
  as_entertank(pnh,"entertank", boost::bind(&InspectionExecution::asEnterTankExecute, this, _1), true),
  as_land(pnh,"land", boost::bind(&InspectionExecution::asLandExecute, this, _1), true),
  as_demopath(pnh,"demopath", boost::bind(&InspectionExecution::asDemoPathExecute, this, _1), true),
  as_mappingtemplate(pnh,"mappingtemplate", boost::bind(&InspectionExecution::asMappingTemplateExecute, this, _1), true) {
		// std::string tmp = nh->getNamespace();
		// ROS_INFO("%s\n", tmp.c_str());
  nh_ = nh;  // Set nodehandle
  pnh_ = pnh;

  // std::cout<<"new version\n";
  rate = &rate1;
  rateHz = 20;
  stage = 1;
  position_tolerance_=0.25;
  // Values to publish for turning LEDs on/off
  led_off_value.data = 0;
  led_on_value.data = 80;

  // Get Params from param server
  if (!pnh_.getParam("requiredCorrectionNo", requiredCorrectionNo))
    requiredCorrectionNo = 4; // movement speed in m/s

  state_sub = nh_->subscribe<mavros_msgs::State>
          ("/mavros/state", 10, &InspectionExecution::state_cb, this);
  local_pos_sub = nh_->subscribe<geometry_msgs::PoseStamped>
          ("/mavros/local_position/pose", 10, &InspectionExecution::pose_cb, this);
  entranceFrontpoint_sub = nh_->subscribe<geometry_msgs::TransformStamped>
          ("/mynteye/hole_front_point", 1, &InspectionExecution::entrance_frontPoint_cb, this);
  local_pos_pub = nh_->advertise<geometry_msgs::PoseStamped>
          ("/mavros/setpoint_position/local", 10);
  led_pub = nh_->advertise<std_msgs::UInt8>
          ("/led_brightness", 10);
  arming_client = nh_->serviceClient<mavros_msgs::CommandBool>
          ("/mavros/cmd/arming");
  set_mode_client = nh_->serviceClient<mavros_msgs::SetMode>
          ("/mavros/set_mode");
  

  // wait for FCU connection
  while(ros::ok() && !current_state.connected || current_pose.pose.orientation.x == 0 || isnan(current_pose.pose.orientation.x)){
      ros::spinOnce();
      rate->sleep();
      // std::cout<<"current_state_connected"<<current_state<<std::endl;
  }
  ros::spinOnce();
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  // Update Robot Params
  updateRobotState();

  // pose2.pose.orientation = pose1.pose.orientation;

  // pose2.pose.position.x = 1.0;
  // pose2.pose.position.z = 2.0;
  lastStablePose = current_pose;

  // mavros_msgs::SetMode offb_set_mode;
  // offb_set_mode.request.custom_mode = "OFFBOARD";

  // mavros_msgs::CommandBool arm_cmd;
  // Initializing command for the arming service to arm/disarm the vehicle
  arm_cmd.request.value = true;
  disarm_cmd.request.value = false;
  

  last_request = ros::Time::now();
  std::cout<<"first curr pose:"<<current_pose<<"\n";

  // TODO: Fix Manual starting of action server not working
  // Start action servers
  // as_takeoff.start();
  // as_publishlaststablepose.start();
  // as_lookforentrance.start();
  
}

void InspectionExecution::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void InspectionExecution::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
}

void InspectionExecution::updateEntranceFrontPoint()
{
    pose_entrance_frontPoint = new_pose_entrance_frontPoint;
}

void InspectionExecution::entrance_frontPoint_cb(const geometry_msgs::TransformStamped tf_w_entrance_frontPoint)
{
    // Only update the entrance when we need to find one
    // if (!look_for_entrance) return;

    // Give correct timestamp
    pose_entrance_frontPoint.header.stamp = tf_w_entrance_frontPoint.header.stamp;

    auto dx = tf_w_entrance_frontPoint.transform.translation.x - lastStablePose.pose.position.x;
    auto dy = tf_w_entrance_frontPoint.transform.translation.y - lastStablePose.pose.position.y;
    auto dz = tf_w_entrance_frontPoint.transform.translation.z - lastStablePose.pose.position.z;
    
    // TODO: investigate how to apply the rotation; can it be just copied from 
    auto curr_yaw =  tf2::getYaw(lastStablePose.pose.orientation);
    // TODO: figure out better way to calculate yaw difference. Offset between world and entrance is pi/2
    auto new_yaw = tf2::getYaw(tf_w_entrance_frontPoint.transform.rotation) + 1.5708;
    auto d_yaw = new_yaw - curr_yaw;
    // std::cout<<"curr_yaw="<<curr_yaw<<"\n new_yaw="<<new_yaw<<"\n d_yaw="<<d_yaw<<std::endl;
    
    geometry_msgs::Quaternion new_quat;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(curr_yaw + d_yaw), new_quat);
    new_pose_entrance_frontPoint.pose.orientation = new_quat;

    new_pose_entrance_frontPoint.pose.position.x = lastStablePose.pose.position.x + dx;
    new_pose_entrance_frontPoint.pose.position.y = lastStablePose.pose.position.y + dy;
    new_pose_entrance_frontPoint.pose.position.z = lastStablePose.pose.position.z + dz;

    // pose_entrance_frontPoint.pose.orientation = lastStablePose.pose.orientation;
    // std::cout<<pose_entrance_frontPoint.pose<<std::endl;
    // Mark that we have a new entrance
    // found_entrance_frontPoint = true;
}

// bool InspectionExecution::moveTo(geometry_msgs::PoseStamped goal, ros::Publisher* pub, double fraction, double sec)
// {
//     if (!calculatedPath)
//     {
//         std::cout<<"Calculating path \n";
//         // If sec not defined, set time equals to difference between positions in meters + 1
//         if (sec == 0)
//             sec = int(2 * inspectrone::EuclDist(goal.pose.position, lastStablePose.pose.position)) + 2;
//         std::cout<<inspectrone::EuclDist(goal.pose.position, lastStablePose.pose.position);
//         std::cout<<"calculated TimeFrame:"<<sec<<std::endl;
//         // V=S/T
//         // T = S/V


//         inspectrone::samplePath(current_pose, goal, sec, rateHz, path, fraction);
//         calculatedPath = true;
//     }

//     auto nextPose = path.front();
//     std::cout<<"Publishing next pose:\n"<<nextPose<<std::endl;
//     pub->publish(nextPose);
//     inMovement = true;
//     path.erase(path.begin());
//     // If path execution is complete
//     if (path.size()>0)
//     {   
//         return false;
//     } 
//     else
//     {
//         calculatedPath = false;
//         lastStablePose = nextPose;
//         stage ++;
//         last_action = ros::Time::now();
//         inMovement = false;
//         return true;
//     };
// }

bool InspectionExecution::moveTo(geometry_msgs::PoseStamped goal, ros::Publisher* pub, double fraction, double sec, std::string mode)
{
    if (!calculatedPath)
    {
        std::cout<<"Calculating path \n";
        //Default mode
        if(mode=="default")
        {
          // If sec not defined, set time equals to difference between positions in meters + 1
          if (sec == 0)
              sec = int(2 * inspectrone::EuclDist(goal.pose.position, lastStablePose.pose.position)) + 2;
          std::cout<<inspectrone::EuclDist(goal.pose.position, lastStablePose.pose.position);
        }
        
        // Constant speed motion mode
        if(mode=="constSpeed")
        {
           sec = (inspectrone::EuclDist(goal.pose.position, lastStablePose.pose.position))/moveSpeed;
        }
        std::cout<<"calculated TimeFrame:"<<sec<<std::endl;
        inspectrone::samplePath(current_pose, goal, sec, rateHz, path, fraction);
        calculatedPath = true;
    }
    
    if(mode=="constSpeed")
    {
      std::cout<<"Movement Speed:"<<moveSpeed<<std::endl;
      std::cout<<"calculated TimeFrame:"<<sec<<std::endl;
    }
    auto nextPose = path.front();
    pub->publish(nextPose);
    inMovement = true;
    path.erase(path.begin());
    // If path execution is complete
    if (path.size()>0)
    {   
        return false;
    } 
    else
    {
        calculatedPath = false;
        lastStablePose = nextPose;
        stage ++;
        last_action = ros::Time::now();
        inMovement = false;
        return true;
    };
}

// Check whether two poses are within position tolerance of eachother
bool InspectionExecution::isPositionReached(geometry_msgs::PoseStamped goal) {
  double dist = inspectrone::EuclDist(goal.pose.position, current_pose.pose.position);

  if (dist <= position_tolerance_) {
    return true;
  } else {
    return false;
  }
}

void InspectionExecution::setCorrectionFlag()
{
    // Let other rosnodes know that we want to correct Entrance's pose
    ros::param::set("/entrance/toCorrectEntrance", true);
    std::cout<<"set /entrance/toCorrectEntrance to true"<<std::endl;
    last_action = ros::Time::now();
    // stage ++;
}

// void InspectionExecution::askToRepeat()
// {
//     if(ros::Time::now() - last_request > ros::Duration(5.0))
//     {
//         last_request = ros::Time::now();
//         std::cout<<"Repeat the entrance approach?, press y to look for entrance again, n to stay here (curr stage:"<<stage<<")\n";
//     }
//     local_pos_pub.publish(lastStablePose);
//     if (kb_input == 'y')
//     {        
//         // found_entrance_frontPoint = false;
//         // look_for_entrance = true;
//         ros::param::set("/entrance/found_valid", false);
//         ros::param::set("/entrance/found_possible", false);
//         // ros::param::set("/entrance/toCorrectEntrance", false);
        
//         stage = 3;
//     } 
//     if (kb_input == 'n') 
//     {      
//         stage ++;
//     }
// }

void InspectionExecution::updateRobotState()
{
  // Update everything that needs to be checked at each run of the behavior!
  ros::param::get("/entrance/found_possible", found_possible_entrance);
  ros::param::get("/entrance/found_valid", found_valid_entrance);
  ros::param::get("/entrance/toCorrectEntrance", toCorrectEntrance);
  // using node handle for moveSpeed as it is being set through the launch file
  // This is done to put the param under global namespace of nodehandle
  if (!pnh_.getParam("moveSpeed", moveSpeed))
    moveSpeed = 0.5; // movement speed in m/s
}

// Take Off action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asTakeOffStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (!as_takeoff_continue) {
    status = false;
  }
  if (as_takeoff.isPreemptRequested()) {
    ROS_WARN("TakeOff action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("TakeOff action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// Take Off action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asTakeOffExecute(const behavior_msgs::TakeOffGoalConstPtr& goal) {
  ROS_INFO("TakeOff action: Starting...");
  as_takeoff_continue = true;
  // Check if any other actions are currently active
  if (as_publishlaststablepose.isActive()) {
    as_publishlaststablepose_continue = false;
    ROS_WARN("TakeOff action: Stopping PublishLastStablePose action...");
  }

  // Check status
  bool status = asTakeOffStatus();
  
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  geometry_msgs::PoseStamped pose_final;

  if (status) {
    try {
      
      status = false;
      // Check if vehicle is armed
      while (ros::ok)
      {
        local_pos_pub.publish(lastStablePose);
        if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
          std::cout<<lastStablePose<<"\n";
          // This line fucks u up
          // if( set_mode_client.call(offb_set_mode) &&
          //    offb_set_mode.response.mode_sent){
          ROS_INFO("Please enable Offboard control");
          // }
          last_request = ros::Time::now();
        } else 
        {
          if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    last_request = ros::Time::now();
                    break;
                }
            }
        }
        ros::spinOnce();
        rate->sleep();
      }
      
      // Take off position
      takeoff_pose.pose.position = current_pose.pose.position;
      takeoff_pose.pose.position.z += 1.0;
      takeoff_pose.pose.orientation = current_pose.pose.orientation;
      // Execute
      while(ros::ok)
      {
        // Move to take off position
        if (moveTo(takeoff_pose, &local_pos_pub))
        {
            std::cout<<"Took off, moving to the next stage\n";
            last_request = ros::Time::now();
            // TODO: remove roll and pitch from a position
            // lastStablePose = takeoff_pose;
            status = true;
            last_action = ros::Time::now();
            break;
            // stage ++;
        }
        ros::spinOnce();
        rate->sleep();
      }
      
      
      // while(!isPositionReached(takeoff_pose)){
      //     ROS_INFO_THROTTLE(1, "Waiting for pose to be reached...");
      //     ros::spinOnce(); // Spin to process odometry callbacks
      // }
     
      // if(status)
      // {
      //   std::cout<<"Took off, moving to the next stage\n";
      //   last_request = ros::Time::now();
      //   // TODO: remove roll and pitch from a position
      //   lastStablePose = takeoff_pose;
      //   last_action = ros::Time::now();
      //   // stage ++;
      // }
      // else
      // {
      //   throw std::runtime_error("Status while moving to take_off pose is false");
      // }
      // TODO: apply logic to wait for take off position to be reached

    } catch(const std::exception& e) {
      ROS_WARN("TakeOff action: Failed with: %s", e.what());
    }
  }
  
  pose_final = current_pose;

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::TakeOffResult msg_result;
  msg_result.success = status;
  msg_result.final_pose = pose_final;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_takeoff.setPreempted(msg_result);
    ROS_WARN("Take Off action: Failed!");
  } 
  else 
  {
    as_takeoff.setSucceeded(msg_result);
  }

  ROS_INFO("Take off action: DONE");

  as_takeoff_continue = false;
}

// CHeck Publish Last Stable Pose action status
bool InspectionExecution::asPublishLastStablePoseStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (!as_publishlaststablepose_continue) {
    status = false;
  }
  if (as_publishlaststablepose.isPreemptRequested()) {
    ROS_WARN("PublishLastStablePose action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("PublishLastStablePose action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// PublishLastStablePose action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asPublishLastStablePoseExecute(const behavior_msgs::PublishLastStablePoseGoalConstPtr& goal) {
  ROS_INFO("PublishLastStablePose action: Starting...");
  as_publishlaststablepose_continue = true;
  // Check if any other actions are currently active
  // if (as_explore.isActive()) {
  //   as_explore_continue = false;
  //   ROS_WARN("Inspect action: Stopping Explore action...");
  // }
  // if (as_gohome.isActive()) {
  //   as_gohome_continue = false;
  //   ROS_WARN("Inspect action: Stopping GoHome action...");
  // }
  // if (as_nextarea.isActive()) {
  //   as_nextarea_continue = false;
  //   ROS_WARN("Inspect action: Stopping NextArea action...");
  // }
  
  bool status = false;

  // Execute all functionality of the mainloop() method
  // apart from the take off
  // while(ros::ok()){
    
    while(ros::ok())
    {   
      kb_input = inspectrone::getch();
      ROS_INFO("Publishing last stable pose! Press 'c' to continue the behavior.");
      std::cout<<"Last stable pose: \n"<<lastStablePose<<std::endl;
      local_pos_pub.publish(lastStablePose);
      // ros::Duration(2.0).sleep();
      if(kb_input == 'c')
      {
        ROS_INFO("C pressed");
        break;
      }
      ros::spinOnce();
      rate->sleep();
    }
    status = true;
    

  // Send result
  
  behavior_msgs::PublishLastStablePoseResult msg_result;
  msg_result.success = status;
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_publishlaststablepose.setPreempted(msg_result);
    ROS_WARN("Publish Last Stable Pose action: Failed!");
  } else {
    as_publishlaststablepose.setSucceeded(msg_result);
  }

  ROS_INFO("Publish Last Stable Pose action: DONE");
  as_publishlaststablepose_continue = false;
}


// Look For Entrance action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asLookForEntranceStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (!as_lookforentrance_continue) {
    status = false;
  }
  if (as_lookforentrance.isPreemptRequested()) {
    ROS_WARN("LookForEntrance action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("LookForEntrance action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// Look For Entrance action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asLookForEntranceExecute(const behavior_msgs::LookForEntranceGoalConstPtr& goal) {
  ROS_INFO("LookForEntrance action: Starting...");
  as_lookforentrance_continue = true;
  // Check if any other actions are currently active
  if (as_publishlaststablepose.isActive()) {
    as_publishlaststablepose_continue = false;
    ROS_WARN("LookForEntrance action: Stopping PublishLastStablePose action...");
  }

  // bool status = true;

  // Check status
  bool status = asLookForEntranceStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      while (ros::ok)
      {
        // Update the rosparams
        updateRobotState();
        // lastStablePose
        double delta_yaw = 0.3/rateHz;
        // TODO: Secure with try catch
        auto nextPose = inspectrone::applyYawToPose(lastStablePose, 0, 0, delta_yaw);
        local_pos_pub.publish(nextPose);
        lastStablePose = nextPose;

        if (found_possible_entrance)
        {
            last_action = ros::Time::now();
            // stage ++;
            // found_entrance_frontPoint = true;
            // look_for_entrance = false;
            status = true;
            break;
        }
        // if (kb_input == 'p')
        // {
        //     last_action = ros::Time::now();
        //     stage --;
        //     paused = true;    
        // } 
       
        ros::spinOnce();
        rate->sleep();
      }
      
      
    } catch(const std::exception& e) {
      ROS_WARN("LookForEntrance action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::LookForEntranceResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_lookforentrance.setPreempted(msg_result);
    ROS_WARN("LookForEntrance action: Failed!");
  } 
  else 
  {
    as_lookforentrance.setSucceeded(msg_result);
  }

  ROS_INFO("Look For Entrance action: DONE");

  as_lookforentrance_continue = false;
}

// Wait To Confirm Entrance action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asWaitToConfirmEntranceStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (as_waittoconfirmentrance.isPreemptRequested()) {
    ROS_WARN("WaitToConfirmEntrance action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("WaitToConfirmEntrance action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// Wait To Confirm Entrance action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asWaitToConfirmEntranceExecute(const behavior_msgs::WaitToConfirmEntranceGoalConstPtr& goal) {
  ROS_INFO("WaitToConfirmEntrance action: Starting...");

  // bool status = true;

  // Check status
  bool status = asWaitToConfirmEntranceStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      while (ros::ok)
      {
         // Update the rosparams
        updateRobotState();
        local_pos_pub.publish(lastStablePose);
        if(ros::Time::now() - last_request > ros::Duration(4.0))
        {
            last_request = ros::Time::now();
            std::cout<<"waitToConfirmEntrance\n";
        }
        // If user confirmed that entrance is valid
        if (found_valid_entrance)
        {
            updateEntranceFrontPoint();
            last_action = ros::Time::now();
            status = true;
            break;
        }
        // When user neglect possible entrance that was found
        if (!found_possible_entrance)
        {
            last_action = ros::Time::now();
            status = false;
            break;
        }
       
        ros::spinOnce();
        rate->sleep();
      }
      
      
    } catch(const std::exception& e) {
      ROS_WARN("WaitToConfirmEntrance action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::WaitToConfirmEntranceResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_waittoconfirmentrance.setPreempted(msg_result);
    ROS_WARN("WaitToConfirmEntrance action: Set Failed by USER! Going to previous stage.");
  } 
  else 
  {
    as_waittoconfirmentrance.setSucceeded(msg_result);
  }

  ROS_INFO("WaitToConfirmEntrance action: DONE");

}


// Wait To Confirm Entrance action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asGetEntranceFrontPointAndMoveStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (as_getentrancefrontpointandmove.isPreemptRequested()) {
    ROS_WARN("GetEntranceFrontPointAndMove action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("GetEntranceFrontPointAndMove action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// Calculate Initial Entrance point action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asGetEntranceFrontPointAndMoveExecute(const behavior_msgs::GetEntranceFrontPointAndMoveGoalConstPtr& goal) {
  ROS_INFO("GetEntranceFrontPointAndMove action: Starting...");

  // bool status = true;

  // Check status
  bool status = asGetEntranceFrontPointAndMoveStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      
      while(ros::ok())
      {
        geometry_msgs::PoseStamped offset;
        offset.pose = pose_entrance_frontPoint.pose;
        offset.pose.position.x = -0.3;
        offset.pose.position.y = 0;
        offset.pose.position.z = 0;
        // std::cout<<"original:"<<pose_entrance_frontPoint.pose<<std::endl;

        geometry_msgs::PoseStamped initialEntrancePoint = 
            inspectrone::calculateRelativePose(pose_entrance_frontPoint, offset);
        pose_entrance_frontPoint.pose = initialEntrancePoint.pose;
        if(moveTo(pose_entrance_frontPoint, &local_pos_pub, 1.0, 5.0, "constSpeed"))
        {
          status = true;
          break;
        }
        ros::spinOnce();
        rate->sleep();
      }
      

      // Print original frontpoint
      // std::cout<<"initial:"<<initialEntrancePoint.pose<<std::endl;
        
    } catch(const std::exception& e) {
      ROS_WARN("GetEntranceFrontPointAndMove action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::GetEntranceFrontPointAndMoveResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_getentrancefrontpointandmove.setPreempted(msg_result);
    ROS_WARN("GetEntranceFrontPointAndMove action: FAILED");
  } 
  else 
  {
    as_getentrancefrontpointandmove.setSucceeded(msg_result);
  }

  ROS_INFO("GetEntranceFrontPointAndMove action: DONE");

}


// CorrectPose action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asCorrectPoseStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (as_correctpose.isPreemptRequested()) {
    ROS_WARN("CorrectPose action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("CorrectPose action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// Calculate Initial Entrance point action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asCorrectPoseExecute(const behavior_msgs::CorrectPoseGoalConstPtr& goal) {
  ROS_INFO("CorrectPose action: Starting...");

  // bool status = true;

  // Check status
  bool status = asCorrectPoseStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      // Let other rosnodes know that we want to correct Entrance's pose
      // Set toCorrectEntrance to true
      setCorrectionFlag();
      // Correct Pose
      while(ros::ok())
      {
        // Update the params
        updateRobotState();
        
        // If you are in moveTo() keep moving
        if(inMovement)
        {
          moveTo(pose_entrance_frontPoint, &local_pos_pub, fraction, 1.0);
          ros::spinOnce();
          rate->sleep();
          continue;
        }
        // MAKE SURE TO NOT PUBLISH AT TWO PLACES
        // Make sure to keep the current pose
        local_pos_pub.publish(lastStablePose);

        // Make sure to keep the current pose
        local_pos_pub.publish(lastStablePose);
        // If time delayed, send update message to a user
        if(ros::Time::now() - last_request > ros::Duration(5.0))
        {
            last_request = ros::Time::now();
            std::cout<<"Correcting pose (curr stage:"<<stage<<")\n";
            // std::cout<<"toCorrectEntrance:"<<toCorrectEntrance<<std::endl;
        }
        // If already corrected no of enough times, enter the Ballast Tank
        if (correctionNo >= requiredCorrectionNo)
        {
          geometry_msgs::PoseStamped offset;
          // TODO: set the quaternion
          offset.pose = lastStablePose.pose;
          offset.pose.position.x = 1.8;
          offset.pose.position.y = 0.0;
          offset.pose.position.z = 0.0;
          std::cout<<"Entering the ballast tank"<<std::endl;

          try
            {
              endPose = inspectrone::calculateRelativePose(lastStablePose, offset);
            }
            catch(const std::exception& e)
            {
              std::cout<<"error in: calculateRelativePose \n";
              std::cerr << e.what() << '\n';
            }
            ros::param::set("/entrance/toCorrectEntrance", false);
            // std::cout<<"set /entrance/toCorrectEntrance to false"<<std::endl;
            last_action = ros::Time::now();
            correctionNo = 0;
            fraction = 1.0; 
            status = true;
            break; 
        }
        // If entrance hasn't been found, keep waiting
        // TODO: Move backward a little?
        if (toCorrectEntrance && inspectrone::passedTime(5.0, last_action) && !inMovement)
        {
            geometry_msgs::PoseStamped offset;
            offset.pose = lastStablePose.pose;
            offset.pose.position.x = -0.1;
            offset.pose.position.y = 0;
            offset.pose.position.z = 0;
            lastStablePose = inspectrone::calculateRelativePose(lastStablePose, offset);
            last_action = ros::Time::now();
        }
        // If entrance has been found. Correct current position (move to a new entrance front point)
        if (!toCorrectEntrance)
        {
            updateEntranceFrontPoint();
            // stage = stage -2;
            last_action = ros::Time::now();
            correctionNo ++;
            fraction = 0.85;
            std::cout<<"Executing correction no:"<<correctionNo<<std::endl;
            moveTo(pose_entrance_frontPoint, &local_pos_pub, fraction, 1.0);
        }
        
        ros::spinOnce();
        rate->sleep();
      }
      

      // Print original frontpoint
      // std::cout<<"initial:"<<initialEntrancePoint.pose<<std::endl;
        
    } catch(const std::exception& e) {
      ROS_WARN("CorrectPose action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::CorrectPoseResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_correctpose.setPreempted(msg_result);
    ROS_WARN("CorrectPose action: FAILED");
  } 
  else 
  {
    as_correctpose.setSucceeded(msg_result);
  }

  ROS_INFO("CorrectPose action: DONE");

}

// EnterTank action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asEnterTankStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (as_entertank.isPreemptRequested()) {
    ROS_WARN("EnterTank action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("EnterTank action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// Calculate Initial Entrance point action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asEnterTankExecute(const behavior_msgs::EnterTankGoalConstPtr& goal) {
  ROS_INFO("EnterTank action: Starting...");

  // bool status = true;

  // Check status
  bool status = asEnterTankStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      led_pub.publish(led_on_value);
      // Move to inside of the tank
      while(ros::ok())
      {
        // Update params
        updateRobotState();
        moveTo(endPose, &local_pos_pub, 1.0, 5.0, "constSpeed");
        if(!inMovement)
        {
          outsideTank = false;
          status = true;
          break;
        }
        
        ros::spinOnce();
        rate->sleep();
      }
      // Enter tank movement succesfull
      // Reset params
      ros::param::set("/entrance/found_valid", false);
      ros::param::set("/entrance/found_possible", false);
      ros::param::set("/entrance/toCorrectEntrance", false);
      correctionNo = 0; 

      // Print original frontpoint
      // std::cout<<"initial:"<<initialEntrancePoint.pose<<std::endl;
        
    } catch(const std::exception& e) {
      ROS_WARN("EnterTank action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::EnterTankResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_entertank.setPreempted(msg_result);
    ROS_WARN("EnterTank action: FAILED");
  } 
  else 
  {
    as_entertank.setSucceeded(msg_result);
  }

  ROS_INFO("EnterTank action: DONE");

}



// Land action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asLandStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (as_land.isPreemptRequested()) {
    ROS_WARN("Land action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("Land action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// Land action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asLandExecute(const behavior_msgs::LandGoalConstPtr& goal) {
  ROS_INFO("Land action: Starting...");

  // bool status = true;

  // Check status
  bool status = asLandStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      landPose = current_pose;
      landPose.pose.position.z = -0.5;
      // Move to inside of the tank
      while(ros::ok())
      {
        // Update params
        updateRobotState();
        moveTo(landPose, &local_pos_pub);
        // When landed, disarm the vehicle
        // if(!inMovement && arming_client.call(disarm_cmd) && disarm_cmd.response.success)
        // {
        //   ROS_INFO("Vehicle disarmed");
        //   last_request = ros::Time::now();
        //   status = true;
        //   break;
        // }

        if(!inMovement)
        {
          status = true;
          break;
        }
        
        ros::spinOnce();
        rate->sleep();
      }
      ROS_INFO("Turning LEDs off");
      led_pub.publish(led_off_value);
      // Enter tank movement succesfull
      

      // Print original frontpoint
      // std::cout<<"initial:"<<initialEntrancePoint.pose<<std::endl;
        
    } catch(const std::exception& e) {
      ROS_WARN("Land action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::LandResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_land.setPreempted(msg_result);
    ROS_WARN("Land action: FAILED");
  } 
  else 
  {
    as_land.setSucceeded(msg_result);
  }

  ROS_INFO("Land action: DONE");

}


// DemoPath action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asDemoPathStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (as_demopath.isPreemptRequested()) {
    ROS_WARN("DemoPath action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("DemoPath action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// DemoPath action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asDemoPathExecute(const behavior_msgs::DemoPathGoalConstPtr& goal) {
  ROS_INFO("DemoPath action: Starting...");

  // bool status = true;

  // Check status
  bool status = asDemoPathStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      geometry_msgs::PoseStamped  offset2, offset3;
      ///// First demo inspection point 
      offset2.pose = lastStablePose.pose;      
      offset2.pose.position.x = 0.8;
      offset2.pose.position.y = 0.2;
      offset2.pose.position.z = 0;
      endPose2 = inspectrone::calculateRelativePose(lastStablePose, offset2);
      endPose2 = inspectrone::applyYawToPose(endPose2, 0, 0, -1.5708);

      ///// Second Demo inspection point
      offset3.pose = lastStablePose.pose;      
      offset3.pose.position.x = -0.2;
      offset3.pose.position.y = 0;
      offset3.pose.position.z = 0;
      endPose3 = inspectrone::calculateRelativePose(endPose2, offset3);
      endPose3 = inspectrone::applyYawToPose(endPose3, 0, 0, 3.1415);

      // Move to first demo point
      while(ros::ok())
      {
        // Update params
        updateRobotState();
        moveTo(endPose2, &local_pos_pub, 1.0, 5);

        if(!inMovement)
        {
          ROS_INFO("First Inspection Point reached!");
          last_action = ros::Time::now();
          break;
        }
        
        ros::spinOnce();
        rate->sleep();
      }
      
      // Wait at first point for 2 seconds
      while(ros::ok())
      {
        // Update params
        updateRobotState();
        //Publish the last stable pose
        local_pos_pub.publish(lastStablePose);
        ROS_INFO_THROTTLE(1,"Waiting to inspect first point");

        if(inspectrone::passedTime(2.0, last_action))
        {
          ROS_INFO("Inspection Done! Going to second inspection point.");
          status = true;
          break;
        }
        
        ros::spinOnce();
        rate->sleep();
      }

      

      // Move to second demo point
      while(ros::ok())
      {
        // Update params
        updateRobotState();
        moveTo(endPose3, &local_pos_pub, 1.0, 5);

        if(!inMovement)
        {
          ROS_INFO("Second Inspection Point reached!");
          last_action = ros::Time::now();
          status = true;
          break;
        }
        
        ros::spinOnce();
        rate->sleep();
      }

      // Wait at second point for 2 seconds
      while(ros::ok())
      {
        // Update params
        updateRobotState();
        //Publish the last stable pose
        local_pos_pub.publish(lastStablePose);
        ROS_INFO_THROTTLE(1,"Waiting to inspect second point");

        if(inspectrone::passedTime(2.0, last_action))
        {
          ROS_INFO("Inspection Done! Going home.");
          status = true;
          break;
        }
        
        ros::spinOnce();
        rate->sleep();
      }
      
    } catch(const std::exception& e) {
      ROS_WARN("DemoPath action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::DemoPathResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_demopath.setPreempted(msg_result);
    ROS_WARN("DemoPath action: FAILED");
  } 
  else 
  {
    as_demopath.setSucceeded(msg_result);
  }

  ROS_INFO("DemoPath action: DONE");

}

// MappingTemplate action - Status check
// Returns true of status is OK, false if not
bool InspectionExecution::asMappingTemplateStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (as_mappingtemplate.isPreemptRequested()) {
    ROS_WARN("MappingTemplate action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("MappingTemplate action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }

  return status;
}

// MappingTemplate action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asMappingTemplateExecute(const behavior_msgs::MappingTemplateGoalConstPtr& goal) {
  ROS_INFO("MappingTemplate action: Starting...");

  // bool status = true;

  // Check status
  bool status = asMappingTemplateStatus();
  if (!status) {
    throw std::runtime_error("Status at start is false");
  }

  if (status) {
    try {
      status = false;
      // Execute Action
      /*




        ENTER ACTION DESCRIPTION HERE 





      */
    } catch(const std::exception& e) {
      ROS_WARN("MappingTemplate action: Failed with: %s", e.what());
    }
  }
  

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::MappingTemplateResult msg_result;
  msg_result.success = status;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_mappingtemplate.setPreempted(msg_result);
    ROS_WARN("MappingTemplate action: FAILED");
  } 
  else 
  {
    as_mappingtemplate.setSucceeded(msg_result);
  }

  ROS_INFO("MappingTemplate action: DONE");

}

