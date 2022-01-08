#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

#include <behavior_msgs/EverythingElseAction.h>
#include <behavior_msgs/TakeOffAction.h>

#include <sensor_msgs/BatteryState.h>

/*
namespace BT {
// Custom type
struct Pose3D
{
  std::string frame; // frame_id
  double px, py, pz, ox, oy, oz, ow; // Position and orientation (quaternion)
};

// Template for automatically converting NodeParameter to Pose3D
template <> inline
Pose3D convertFromString(StringView key)
{
    // Eight parts - 1 string and 7 real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 8)
    {
        throw BT::RuntimeError("invalid input");
    }
    else
    {
        Pose3D output;
        output.frame = convertFromString<std::string>(parts[0]);
        output.px    = convertFromString<double>(parts[1]);
        output.py    = convertFromString<double>(parts[2]);
        output.pz    = convertFromString<double>(parts[3]);
        output.ox    = convertFromString<double>(parts[4]);
        output.oy    = convertFromString<double>(parts[5]);
        output.oz    = convertFromString<double>(parts[6]);
        output.ow    = convertFromString<double>(parts[7]);
        return output;
    }
}
} // end namespace BT
*/



namespace MAVInspectionNodes {

std::string execution_ns_ = "/behavior_node";

sensor_msgs::BatteryState *batt_ptr;
double batt_limit = 20.0; // Remaining capacity percentage

bool *cont_ptr;
void setBatteryPtr(sensor_msgs::BatteryState *battery_msg_ptr);
void setBatteryLimit(double limit);
void setContinuePtr(bool *continue_ptr);

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// CONDITIONS ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

BT::NodeStatus CheckContinue();
BT::NodeStatus CheckBattery();

/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// ACTIONS ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// Take Off action ////////////////////////////////////////

// Asynchronous action
class TakeOffAction : public BT::AsyncActionNode
{
public:
  // Any TreeNode with ports must have a constructor with this signature
  TakeOffAction(const std::string &name, const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config),
      ac_(execution_ns_ + "/takeoff", true)
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    // return {BT::InputPort<double>("timeout")};
  }
  //The init() method below is used to send arguments which change at run time, to an action 
  // We want this method to be called ONCE and BEFORE the first tick()
  // void init( geometry_msgs::PoseStamped moveto_goal_pose, double fraction, double sec)
  //   // void init( *tank_entrance::MoveToParams paramPtr)
  //   {
  //       _moveto_goal_pose = (moveto_goal_pose);
  //       _fraction = (fraction);
  //       _sec = (sec);
  //   }

  BT::NodeStatus tick() override;

  virtual void halt() override;

  void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::TakeOffResultConstPtr& result);

private:
  actionlib::SimpleActionClient<behavior_msgs::TakeOffAction> ac_;
  bool ac_done = false;
  bool ac_success = false;
  std::atomic_bool _halt_requested;
  // geometry_msgs::PoseStamped _moveto_goal_pose;
  // double _fraction;
  // double _sec;

};


//////////////////////////////////////// Everything Else action ////////////////////////////////////////

// Asynchronous action
class EverythingElseAction : public BT::AsyncActionNode
{
public:
  // Any TreeNode with ports must have a constructor with this signature
  EverythingElseAction(const std::string &name, const BT::NodeConfiguration &config)
      : AsyncActionNode(name, config),
      ac_(execution_ns_ + "/everythingelse", true)
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    // return {BT::InputPort<double>("timeout")};
  }

  BT::NodeStatus tick() override;

  virtual void halt() override;

  void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::EverythingElseResultConstPtr& result);

private:
  actionlib::SimpleActionClient<behavior_msgs::EverythingElseAction> ac_;
  bool ac_done = false;
  bool ac_success = false;
  std::atomic_bool _halt_requested;
};


} // end namespace MAVInspectionNodes