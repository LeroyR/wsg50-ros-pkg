#ifndef GRIPPER_ACTION_WSG_H
#define GRIPPER_ACTION_WSG_H

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/GripperCommandAction.h>

// actionlib
#include <actionlib/server/action_server.h>

#include "wsg_50_common/Status.h"
#include "wsg_50_common/Cmd.h"

namespace gripper_command_action
{
class GripperActionController
{
  public:
    typedef boost::shared_ptr<GripperActionController> Ptr;
    struct Commands
    {
        double position_;   // Last commanded position
        double max_effort_; // Max allowed effort
    };

    GripperActionController();

    void run()
    {
      ros::spin();
    }

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    /*\}*/

    void update(const ros::Time &time, const ros::Duration &period);
    /*\}*/

    Commands command_struct_; // pre-allocated memory

  private:
    typedef actionlib::ActionServer<control_msgs::GripperCommandAction> ActionServer;
    typedef boost::shared_ptr<ActionServer> ActionServerPtr;
    typedef ActionServer::GoalHandle GoalHandle;

    bool update_hold_position_;

    bool verbose_;
    std::string name_;
    std::string joint_name_;

    control_msgs::GripperCommandResultPtr pre_alloc_result_;

    ros::Duration action_monitor_period_;

    // ROS API
    ros::NodeHandle controller_nh_;
    ActionServerPtr action_server_;
    GoalHandle active_goal_;
    bool has_active_goal_ = false;



    // WSG APi
    ros::Subscriber sub_gripper_state_;
    ros::Publisher pub_gripper_command_;
    bool state_recvd_ = false;
    wsg_50_common::StatusPtr last_gripper_state_;

    ros::Timer goal_handle_timer_;

    void goalCB(GoalHandle gh);
    void cancelCB(GoalHandle gh);
    void preemptActiveGoal();
    void setHoldPosition(const ros::Time &time);
    void controllerStateCB(const wsg_50_common::StatusPtr &msg);

    ros::Time last_movement_time_;

    double stall_timeout_, stall_velocity_threshold_, last_vel;
    double default_max_effort_;
    double goal_tolerance_;
    void checkForSuccess(const ros::Time &time, double error_position, double current_position, double current_velocity);
};
}
#endif // header guard