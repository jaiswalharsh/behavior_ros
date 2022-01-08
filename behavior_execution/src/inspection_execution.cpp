#include "behavior_execution/inspection_execution.hpp"


// Destructor
InspectionExecution::~InspectionExecution() {
  ROS_INFO("Destructing InspectionExecution...");
  // Free up allocated memory
  // delete tf_listener_ptr_;
  // delete tf_broadcaster_ptr_;
}

// Constructor
InspectionExecution::InspectionExecution(ros::NodeHandle* nh, ros::NodeHandle pnh, ros::Rate rate1) : 
  as_everythingelse(pnh, "everythingelse", boost::bind(&InspectionExecution::asEverythingElseExecute, this, _1), false), 
  as_takeoff(pnh, "takeoff", boost::bind(&InspectionExecution::asTakeOffExecute, this, _1), false) {
    //   nh_ = nh;  // Set nodehandle
    //   pnh_ = pnh;

    // ##############################constructor copied from tank entrance
  std::cout<<"new version\n";
  rate = &rate1;
  rateHz = 20;
  stage = 1;

  state_sub = nh->subscribe<mavros_msgs::State>
          ("mavros/state", 10, &InspectionExecution::state_cb, this);
  local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>
          ("/mavros/local_position/pose", 10, &InspectionExecution::pose_cb, this);
  entranceFrontpoint_sub = nh->subscribe<geometry_msgs::TransformStamped>
          ("/mynteye/hole_front_point", 1, &InspectionExecution::entrance_frontPoint_cb, this);
  local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);
  arming_client = nh->serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  set_mode_client = nh->serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  // wait for FCU connection
  while(ros::ok() && !current_state.connected || current_pose.pose.orientation.x == 0 || isnan(current_pose.pose.orientation.x)){
      ros::spinOnce();
      rate->sleep();
  }
  ros::spinOnce();
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  
  // pose2.pose.orientation = pose1.pose.orientation;

  // pose2.pose.position.x = 1.0;
  // pose2.pose.position.z = 2.0;
  lastStablePose = current_pose;

  //send a few setpoints before starting
  for(int i = 50; ros::ok() && i > 0; --i){
      local_pos_pub.publish(takeoff_pose);
      ros::spinOnce();
      rate->sleep();
  }

  // mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // mavros_msgs::CommandBool arm_cmd;
  // Requesting the arming service to arm the vehicle
  arm_cmd.request.value = true;

  last_request = ros::Time::now();
  std::cout<<current_pose<<"\n";

  // tank_entrance::mainLoop();

  // ######################## Tank entrance constructor ends


  // Start action servers
  as_takeoff.start();
  as_everythingelse.start();
  
}

// ##################tank entrance functions start here##################

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

bool InspectionExecution::moveTo(geometry_msgs::PoseStamped goal, ros::Publisher* pub, double fraction, double sec)
{
    if (!calculatedPath)
    {
        std::cout<<"Calculating path \n";
        // If sec not defined, set time equals to difference between positions in meters + 1
        if (sec == 0)
            sec = int(2 * inspectrone::EuclDist(goal.pose.position, lastStablePose.pose.position)) + 2;
        std::cout<<inspectrone::EuclDist(goal.pose.position, lastStablePose.pose.position);
        std::cout<<"calculated TimeFrame:"<<sec<<std::endl;

        inspectrone::samplePath(current_pose, goal, sec, rateHz, path, fraction);
        calculatedPath = true;
    }
    auto nextPose = path.front();
    pub->publish(nextPose);
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
        return true;
    };
}


void InspectionExecution::holdPositionIfPaused()
{
    local_pos_pub.publish(lastStablePose);
    // If pause was initiated in any other stage, then wait until user input
    if (paused)
    {
        if(ros::Time::now() - last_request > ros::Duration(5.0))
        {
            last_request = ros::Time::now();
            std::cout<<"holding pose, press y to move to the next stage (curr stage:"<<stage<<")\n";
        }
        local_pos_pub.publish(lastStablePose);
        if (kb_input == 'y') 
        {
            last_action = ros::Time::now();
            paused = false;
            stage ++;
        }
    }
    else
    {
        if(ros::Time::now() - last_request > ros::Duration(2.0))
        {
            last_request = ros::Time::now();
            stage++;
            return;
            // std::cout<<"holding pose, press y to move to the next stage (curr stage:"<<stage<<")\n";
        }
        if (kb_input == 'y') stage ++;
    }
}

void InspectionExecution::confirmEntrance()
{
    if(ros::Time::now() - last_request > ros::Duration(4.0))
    {
        last_request = ros::Time::now();
        std::cout<<"pose_entrance_frontPoint: \n"<<pose_entrance_frontPoint<<std::endl;
        std::cout<<"please confirm that the entrance is valid. Press y to move to the entrance front point, n to keep looking (curr stage:"<<stage<<")\n";
    }
    local_pos_pub.publish(lastStablePose);

    if (kb_input == 'y') 
    {
        last_action = ros::Time::now();
        stage ++;
    }
    if (kb_input == 'n') 
    {
        last_action = ros::Time::now();
        // found_entrance_frontPoint = false;
        // look_for_entrance = true;
        stage --;
    }
}

void InspectionExecution::waitToConfirmEntrance()
{
    local_pos_pub.publish(lastStablePose);
    if(ros::Time::now() - last_request > ros::Duration(4.0))
    {
        last_request = ros::Time::now();
        std::cout<<"waitToConfirmEntrance (curr stage:"<<stage<<")\n";
    }
    // If user confirmed that entrance is valid
    if (found_valid_entrance)
    {
        updateEntranceFrontPoint();
        last_action = ros::Time::now();
        stage++;
        return;
    }
    // When user neglect possible entrance that was found
    if (!found_possible_entrance)
    {
        last_action = ros::Time::now();
        stage--;
        found_entrance_frontPoint = false;
        // look_for_entrance = true;
        return;
    }
    return;
}

void InspectionExecution::lookForEntrance()
{
    // look_for_entrance = true;
    if(ros::Time::now() - last_request > ros::Duration(5.0))
    {
        last_request = ros::Time::now();
        std::cout<<"looking for an entrance, press p to pause rotation (move back to prev stage) (curr stage:"<<stage<<")\n";
    }
    // lastStablePose
    double delta_yaw = 0.1/rateHz;
    // TODO: Secure with try catch
    auto nextPose = inspectrone::applyYawToPose(lastStablePose, 0, 0, delta_yaw);
    local_pos_pub.publish(nextPose);
    lastStablePose = nextPose;

    if (found_possible_entrance)
    {
        last_action = ros::Time::now();
        stage ++;
        // found_entrance_frontPoint = true;
        // look_for_entrance = false;
        return;
    }
    if (kb_input == 'p')
    {
        last_action = ros::Time::now();
        stage --;
        paused = true;    
    } 
}

void InspectionExecution::correctPose()
{
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
        geometry_msgs::PoseStamped offset, offset2;
        // TODO: set the quaternion
        offset.pose = lastStablePose.pose;
        offset.pose.position.x = 1.8;
        offset.pose.position.y = 0.0;
        offset.pose.position.z = 0.0;
        std::cout<<"Entering the ballast tank"<<std::endl;

        try
        {
            endPose = inspectrone::calculateRelativePose(lastStablePose, offset);
            offset2.pose = endPose.pose;
            if (outsideTank)
            {
                offset2.pose.position.y = 1;
            }
            else{
                offset2.pose.position.y = -1;
            }        
            offset2.pose.position.x = 1.45;
            offset2.pose.position.z = 0;
            endPose2 = inspectrone::calculateRelativePose(endPose, offset2);
            endPose2 = inspectrone::applyYawToPose(endPose2, 0, 0, 2.5);
            outsideTank = false;
        }
        catch(const std::exception& e)
        {
            std::cout<<"error in: calculateRelativePose \n";
            std::cerr << e.what() << '\n';
            // Go to default mode
            stage = -1;
        }
        ros::param::set("/entrance/toCorrectEntrance", false);
        // std::cout<<"set /entrance/toCorrectEntrance to false"<<std::endl;
        last_action = ros::Time::now();
        correctionNo = 0;
        fraction = 1.0;
        stage++;
        return;
    }
    // If entrance hasn't been found, keep waiting
    // TODO: Move backward a little?
    if (toCorrectEntrance && inspectrone::passedTime(5.0, last_action))
    {
        geometry_msgs::PoseStamped offset;
        offset.pose = lastStablePose.pose;
        offset.pose.position.x = -0.1;
        offset.pose.position.y = 0;
        offset.pose.position.z = 0;
        lastStablePose = inspectrone::calculateRelativePose(lastStablePose, offset);

        last_action = ros::Time::now();
        return;
    }
    // If entrance has been found. Correct current position (move to a new entrance front point)
    if (!toCorrectEntrance)
    {
        updateEntranceFrontPoint();
        stage = stage -2;
        last_action = ros::Time::now();
        correctionNo ++;
        fraction = 0.85;
        std::cout<<"Executing correction no:"<<correctionNo<<std::endl;
        return;
    }
}

void InspectionExecution::setCorrectionFlag()
{
    // Let other rosnodes know that we want to correct Entrance's pose
    ros::param::set("/entrance/toCorrectEntrance", true);
    std::cout<<"set /entrance/toCorrectEntrance to true"<<std::endl;
    last_action = ros::Time::now();
    stage ++;
}

void InspectionExecution::calculateInitialEntrancePoint()
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

    // Print original frontpoint
    // std::cout<<"initial:"<<initialEntrancePoint.pose<<std::endl;
    
    stage ++;

    return;
}

void InspectionExecution::askToRepeat()
{
    if(ros::Time::now() - last_request > ros::Duration(5.0))
    {
        last_request = ros::Time::now();
        std::cout<<"Repeat the entrance approach?, press y to look for entrance again, n to stay here (curr stage:"<<stage<<")\n";
    }
    local_pos_pub.publish(lastStablePose);
    if (kb_input == 'y')
    {        
        // found_entrance_frontPoint = false;
        // look_for_entrance = true;
        ros::param::set("/entrance/found_valid", false);
        ros::param::set("/entrance/found_possible_entrance", false);
        // ros::param::set("/entrance/toCorrectEntrance", false);
        
        stage = 3;
    } 
    if (kb_input == 'n') 
    {      
        stage ++;
    }
}

// TODO: move to inspectrone library
bool InspectionExecution::kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

char InspectionExecution::getch() 
{
    if (!kbhit())
        return 0;
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
    return (buf);
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
}

// Take Off action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asTakeOffExecute(const behavior_msgs::TakeOffGoalConstPtr& goal) {
  ROS_INFO("TakeOff action: Starting...");
  as_takeoff_continue = true;
  // Check if any other actions are currently active
  // if (as_explore.isActive()) {
  //   as_explore_continue = false;
  //   ROS_WARN("TakeOff action: Stopping Explore action...");
  // }
  // if (as_inspect.isActive()) {
  //   as_inspect_continue = false;
  //   ROS_WARN("TakeOff action: Stopping Inspect action...");
  // }
  // if (as_nextarea.isActive()) {
  //   as_nextarea_continue = false;
  //   ROS_WARN("TakeOff action: Stopping NextArea action...");
  // }

  bool status = true;

  geometry_msgs::PoseStamped pose_final;

  if (status) {
    try {
      // Check status
      status = asTakeOffStatus();
      if (!status) {
        throw std::runtime_error("Status at start is false");
      }
      
      // Take off position
      takeoff_pose.pose.position = current_pose.pose.position;
      takeoff_pose.pose.position.z += 1;
      takeoff_pose.pose.orientation = current_pose.pose.orientation;
      // Execute
      
      status = moveTo(takeoff_pose, &local_pos_pub, 2);
      
      if(status)
      {
        std::cout<<"Took off, moving to the next stage\n";
        last_request = ros::Time::now();
        // TODO: remove roll and pitch from a position
        lastStablePose = takeoff_pose;
        last_action = ros::Time::now();
        // stage ++;
      }
      else
      {
        throw std::runtime_error("Status while moving to take_off pose is false");
      }
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
    ROS_WARN("Go home action: Failed!");
  } else {
    as_takeoff.setSucceeded(msg_result);
  }

  ROS_INFO("Go home action: DONE");

  as_takeoff_continue = false;
}

// CHeck everything else action status
bool InspectionExecution::asEverythingElseStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (!as_everythingelse_continue) {
    status = false;
  }
  if (as_everythingelse.isPreemptRequested()) {
    ROS_WARN("EverythingElse action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("EverythingElse action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }
}

// EverythingElse action - Running in its own thread (Implemented in SimpleActionServer?)
void InspectionExecution::asEverythingElseExecute(const behavior_msgs::EverythingElseGoalConstPtr& goal) {
  ROS_INFO("Everything Else action: Starting...");
  as_everythingelse_continue = true;
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
  
  bool status = true;

  // Execute all functionality of the mainloop() method
  // apart from the take off
  while(ros::ok()){
    // Update the rosparams
    ros::param::get("/entrance/found_valid", found_valid_entrance);
    ros::param::get("/entrance/found_possible", found_possible_entrance);
    ros::param::get("/entrance/toCorrectEntrance", toCorrectEntrance);
    // ros::param::get("/entrance/found_possible", found_entrance_frontPoint);
    
    kb_input = getch();
    if (kb_input != 0)
        std::cout<<kb_input<<"\n";

    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
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
            }
            last_request = ros::Time::now();
        }
    }
    //  From this point on the quad is armed and ready for code execution
    switch (stage)
    {
        // Take off
        case 1:
            // takeoff(pose1);
            break;
        // Hold last stable position
        case 2:
            holdPositionIfPaused();
            break;
        case 3:
            // Rotate around
            lookForEntrance();
            break;
        case 4:
            // Wait until user confirm entrance and set it to "valid"
            waitToConfirmEntrance();
            break;
        case 5:
            // Calculate initial waypoint that is slightly further from the entrance than the remaining ones
            calculateInitialEntrancePoint();
            break;
        case 6:
            // New method that calculates time of flight based on the distance
            moveTo(pose_entrance_frontPoint, &local_pos_pub, fraction); 
            break;
        case 7:
            // Set toCorrectEntrance to true
            setCorrectionFlag();
            break;
        case 8:
            // Wait till toCorrectEntrance will be set to false by HoleDetection (read entrance detected)
            correctPose();
            break;  
        case 9:
            moveTo(endPose, &local_pos_pub, 1.0, 5);
            break;  
        case 10:
            moveTo(endPose2, &local_pos_pub, 1.0, 10);
            break;  
        default:
            askToRepeat();
            break;
    }
    ros::spinOnce();
    rate->sleep();
  }

  // Send result
  
  behavior_msgs::EverythingElseResult msg_result;
  msg_result.success = status;
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_everythingelse.setPreempted(msg_result);
    ROS_WARN("Inspect action: Failed!");
  } else {
    as_everythingelse.setSucceeded(msg_result);
  }

  ROS_INFO("Everything Else action: DONE");
  as_everythingelse_continue = false;
}

