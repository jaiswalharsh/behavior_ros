#include "behavior_execution/inspection_execution.hpp"
// #include <tank_entrance/enter_tank_bt_test.hpp>

// Constructor
BehaviourExecution::BehaviourExecution(ros::NodeHandle& nh, ros::NodeHandle& pnh) : 
  as_moveto(pnh, "moveto", boost::bind(&BehaviourExecution::asMoveToExecute, this, _1), false) {
  nh_ = nh;  // Set nodehandle
  pnh_ = pnh;

  // Initialise variables / parameters to class variables


  // Initialise publishers and subscribers
  // local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>
  //           ("mavros/setpoint_position/local", 10);

  // Start action servers
  as_moveto.start();
}

// Destructor
BehaviourExecution::~BehaviourExecution() {
  ROS_INFO("Destructing BehaviourExecution...");
  // Free up allocated memory
}

// bool BehaviourExecution::computePathAndMove(geometry_msgs::PoseStamped goal, ros::Publisher* pub, double fraction, double sec)
// {
//     if (!tank_entrance::calculatedPath)
//     {
//         std::cout<<"Calculating path \n";
//         // If sec not defined, set time equals to difference between positions in meters + 1
//         if (sec == 0)
//             sec = int(2 * inspectrone::EuclDist(goal.pose.position, tank_entrance::lastStablePose.pose.position)) + 2;
//         std::cout<<inspectrone::EuclDist(goal.pose.position, tank_entrance::lastStablePose.pose.position);
//         std::cout<<"calculated TimeFrame:"<<sec<<std::endl;


//         // Why dont we use a return variable in the samplePath function?
//         inspectrone::samplePath(tank_entrance::current_pose, goal, sec, tank_entrance::rateHz, tank_entrance::path, fraction);
//         tank_entrance::calculatedPath = true;
//     }

//     auto nextPose = path.front();
//     pub->publish(nextPose);
//     path.erase(path.begin());
//     // If path execution is complete
//     if (path.size()>0)
//     {
//         return false;
//     } 
//     else
//     {
//         tank_entrance::calculatedPath = false;
//         tank_entrance::lastStablePose = nextPose;
//         tank_entrance::stage ++;
//         tank_entrance::last_action = ros::Time::now();
//         return true;
//     };
// }


// MoveTo action - Status check
// Returns true of status is OK, false if not
bool BehaviourExecution::asMoveToStatus() {
  bool status = true;
  // Check if preempt has been requested by the client, or ROS is down
  if (!as_moveto_continue) {
    status = false;
  }
  if (as_moveto.isPreemptRequested()) {
    ROS_WARN("MoveTo action: Preempted requested");
    status = false;
  }
  if (!ros::ok()) {
    ROS_WARN("MoveTo action: ROS is down");
    status = false;
  }
  
  // Handle status == false
  if (status == false) {
    // 
  }
}


// Go Home action - Running in its own thread (Implemented in SimpleActionServer?)
// FIXME: Currently ignoring holes - just computes a global path through the area seen so far
//        Ideally should be implemented as a shortest path problem through areas that have been explored so far
void BehaviourExecution::asMoveToExecute(const behavior_msgs::MoveToGoalConstPtr& goal) {
  ROS_INFO("MoveTo action: Starting...");
  as_moveto_continue = true;
  // Check if any other actions are currently active

  bool status = true;
  // Go to manual mode

  geometry_msgs::PoseStamped pose_final;

  if (status) {
    try {
      // Check status
      status = asMoveToStatus();
      if (!status) {
        throw std::runtime_error("Status at start is false");
      }

      status = tank_entrance::moveTo(goal.goal_pose,&tank_entrance::local_pos_pub,goal.fraction,goal.sec);

      //TODO: Implement method to check if final position reached
  

    } catch(const std::exception& e) {
      ROS_WARN("MoveTo action: Failed with: %s", e.what());
    }
  }
  
  // pose_final = <Call Get current pose method from tank entrance >

  // Send feedback
  // This action has no feedback

  // Send result
  behavior_msgs::MoveToResult msg_result;
  msg_result.success = status;
  msg_result.final_pose = pose_final;
  
  // Handle status
  if (!status) { // Preempted or shut down
    // Preempt
    as_moveto.setPreempted(msg_result);
    ROS_WARN("Move to action: Failed!");
  } else {
    as_moveto.setSucceeded(msg_result);
  }

  ROS_INFO("Move to action: DONE");

  as_moveto_continue = false;
}