#include "wsg_50/gripper_command_action.h"
#include "wsg_50/specialized_gripper_command_action.h"

namespace gripper_command_action
{
SpecializedGripperActionController::SpecializedGripperActionController(std::string action_name) : action_name_(action_name)
{
}

std::string getLeafNamespace(const ros::NodeHandle &nh)
{
    const std::string complete_ns = nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    return complete_ns.substr(id + 1);
}

bool SpecializedGripperActionController::init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{

    // Cache controller node handle
    controller_nh_ = controller_nh;

    // Controller name
    name_ = getLeafNamespace(controller_nh_);
    // Action status checking update rate
    double action_monitor_rate = 20.0;
    controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
    action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
    ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

    // Controlled joint
    controller_nh_.getParam("joint", joint_name_);
    if (joint_name_.empty())
    {
        ROS_ERROR_STREAM_NAMED(name_, "Could not find joint name on param server");
        return false;
    }

    // Default tolerances
    controller_nh_.param<double>("goal_tolerance", goal_tolerance_, 0.001);
    goal_tolerance_ = fabs(goal_tolerance_);
    // Max allowable effort
    controller_nh_.param<double>("max_effort", default_max_effort_, 0.0);
    default_max_effort_ = fabs(default_max_effort_);
    // Stall - stall velocity threshold, stall timeout
    controller_nh_.param<double>("stall_velocity_threshold", stall_velocity_threshold_, 0.001);
    controller_nh_.param<double>("stall_timeout", stall_timeout_, 1.0);
    // Command
    command_struct_.position_ = 0.0;
    command_struct_.max_effort_ = default_max_effort_;
    // Result
    pre_alloc_result_.reset(new wsg_50_common::WeissGripperCmdResult());
    pre_alloc_result_->position = command_struct_.position_;
    pre_alloc_result_->reached_goal = false;
    pre_alloc_result_->stalled = false;
    pre_alloc_result_->status = "UNKNOWN";

    // Gripper state and command
    pub_gripper_command_ = controller_nh_.advertise<wsg_50_common::StateCmd>(
        "goal_command", 1);
    sub_gripper_state_ = controller_nh_.subscribe("status", 1, &SpecializedGripperActionController::controllerStateCB, this);
    // ROS API: Action interface
    action_server_.reset(new ActionServer(controller_nh_, action_name_,
                                          boost::bind(&SpecializedGripperActionController::goalCB, this, _1),
                                          boost::bind(&SpecializedGripperActionController::cancelCB, this, _1),
                                          false));
    action_server_->start();
    return true;
}

void SpecializedGripperActionController::goalCB(GoalHandle gh)
{
    if (has_active_goal_)
    {
        ROS_WARN("Rejected action goal request, because server has an ongoing action!");
        gh.setRejected();
    }
    else
    {
        ROS_INFO("Received action goal request");
        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;
        pub_gripper_command_.publish(gh.getGoal()->cmd);
        last_movement_time_ = ros::Time::now();
    }
}

void SpecializedGripperActionController::cancelCB(GoalHandle gh)
{
    ROS_WARN("Received action cancel request");
    // Check that cancel request refers to currently active goal (if any)
    if (has_active_goal_ && active_goal_ == gh)
    {
        // Reset current goal

        // Enter hold current position mode
        setHoldPosition(ros::Time(0.0));
        ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");

        // Mark the current goal as canceled
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }
}

void SpecializedGripperActionController::preemptActiveGoal()
{
    // Cancels the currently active goal
    if (has_active_goal_)
    {
        // Marks the current goal as canceled
        if (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        {
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }
    }
}

void SpecializedGripperActionController::setHoldPosition(const ros::Time &time)
{
    command_struct_.position_ = last_gripper_state_->width / 2; //SET TO CURRENT POSITION
    command_struct_.max_effort_ = default_max_effort_;
    wsg_50_common::StateCmd command_msg;
    command_msg.pos = command_struct_.position_;
    command_msg.command = command_msg.STOP;
    pub_gripper_command_.publish(command_msg);
}

void SpecializedGripperActionController::checkForSuccess(const ros::Time &time, double error_position, double current_position, double current_velocity)
{
    if (!has_active_goal_)
        return;

    if (active_goal_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
        return;

    if (fabs(error_position) < goal_tolerance_)
    {
        pre_alloc_result_->effort = last_gripper_state_->force;
        pre_alloc_result_->position = current_position;
        pre_alloc_result_->reached_goal = true;
        pre_alloc_result_->stalled = false;
        pre_alloc_result_->status = last_gripper_state_->status;

        active_goal_.setSucceeded(*pre_alloc_result_);
        has_active_goal_ = false;
    }
    else
    {
        if (fabs(current_velocity) > stall_velocity_threshold_)
        {
            ROS_INFO_STREAM("no stall " << fabs(current_velocity));
            last_movement_time_ = time;
        }
        else if ((time - last_movement_time_).toSec() > stall_timeout_)
        {
            pre_alloc_result_->effort = last_gripper_state_->force;
            pre_alloc_result_->position = current_position;
            pre_alloc_result_->reached_goal = false;
            pre_alloc_result_->stalled = true;
            pre_alloc_result_->status = last_gripper_state_->status;
            active_goal_.setAborted(*pre_alloc_result_);
            has_active_goal_ = false;
        }
        else
        {
            ROS_INFO_STREAM("current_position  " << current_position << " goal_tolerance: " << goal_tolerance_ << " error_position: " << fabs(error_position) << " velocity: " << fabs(current_velocity));
        }
    }
}

void SpecializedGripperActionController::update(const ros::Time &time, const ros::Duration &period)
{
    if (state_recvd_)
    {
        double current_position = last_gripper_state_->width / 1000;
        double current_velocity = last_vel;
        double error_position = command_struct_.position_ - current_position;
        double error_velocity = -current_velocity;
        checkForSuccess(time, error_position, current_position, current_velocity);
        state_recvd_ = false;
    }
    else
    {
        ROS_DEBUG("no new state available");
    }
}

void SpecializedGripperActionController::controllerStateCB(
    const wsg_50_common::StatusPtr &msg)
{
    ROS_DEBUG("Checking controller state feedback");
    if (!last_gripper_state_)
        last_gripper_state_ = msg;
    last_vel = (msg->width - last_gripper_state_->width) / 2;
    last_gripper_state_ = msg;
    state_recvd_ = true;
}

std::string SpecializedGripperActionController::getTopic()
{
    return std::string(this->controller_nh_.getNamespace() + "/" + this->action_name_);
}
}