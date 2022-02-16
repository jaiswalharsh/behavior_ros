#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

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

  std::string execution_ns_ = "/inspection/inspection_node";

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
        ac_(execution_ns_ +  "/takeoff", true)
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
  class PublishLastStablePoseAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    PublishLastStablePoseAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ + "/publishlaststablepose", true)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
      // return {BT::InputPort<double>("timeout")};
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::PublishLastStablePoseResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::PublishLastStablePoseAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
  };

  //////////////////////////////////////// Look For Entrance action ////////////////////////////////////////

  // Asynchronous action
  class LookForEntranceAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    LookForEntranceAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/lookforentrance", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::LookForEntranceResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::LookForEntranceAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };

  //////////////////////////////////////// Wait To Confirm Entrance action ////////////////////////////////////////

  // Asynchronous action
  class WaitToConfirmEntranceAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    WaitToConfirmEntranceAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/waittoconfirmentrance", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::WaitToConfirmEntranceResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::WaitToConfirmEntranceAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };

    //////////////////////////////////////// Calculate Initial Entrance Point action ////////////////////////////////////////

  // Asynchronous action
  class GetEntranceFrontPointAndMoveAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    GetEntranceFrontPointAndMoveAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/getentrancefrontpointandmove", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::GetEntranceFrontPointAndMoveResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::GetEntranceFrontPointAndMoveAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };

  //////////////////////////////////////// Correct Pose action ////////////////////////////////////////

  // Asynchronous action
  class CorrectPoseAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    CorrectPoseAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/correctpose", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::CorrectPoseResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::CorrectPoseAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };


  //////////////////////////////////////// Correct Pose action ////////////////////////////////////////

  // Asynchronous action
  class EnterTankAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    EnterTankAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/entertank", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::EnterTankResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::EnterTankAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };


  //////////////////////////////////////// Land action ////////////////////////////////////////

  // Asynchronous action
  class LandAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    LandAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/land", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::LandResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::LandAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };


//////////////////////////////////////// Demo Path action ////////////////////////////////////////

  // Asynchronous action
  class DemoPathAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    DemoPathAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/demopath", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::DemoPathResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::DemoPathAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };


  //////////////////////////////////////// Mapping Template action ////////////////////////////////////////

  // Asynchronous action
  class MappingTemplateAction : public BT::AsyncActionNode
  {
  public:
    // Any TreeNode with ports must have a constructor with this signature
    MappingTemplateAction(const std::string &name, const BT::NodeConfiguration &config)
        : AsyncActionNode(name, config),
        ac_(execution_ns_ +  "/mappingtemplate", true)
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

    void doneCB(const actionlib::SimpleClientGoalState& state, const behavior_msgs::MappingTemplateResultConstPtr& result);

  private:
    actionlib::SimpleActionClient<behavior_msgs::MappingTemplateAction> ac_;
    bool ac_done = false;
    bool ac_success = false;
    std::atomic_bool _halt_requested;
    // geometry_msgs::PoseStamped _moveto_goal_pose;
    // double _fraction;
    // double _sec;

  };


} // end namespace MAVInspectionNodes