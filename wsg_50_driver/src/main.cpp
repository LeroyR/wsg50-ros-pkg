/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief WSG-50 ROS driver.
 */

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <xamla_sysmon_msgs/statuscodes.h>
#include <xamla_sysmon_msgs/HeartBeat.h>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"
#include "wsg_50/specialized_gripper_command_action.h"
#include "wsg_50/gripper_command_action.h"
#include "wsg_50/gripper_monitor.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Conf.h"
#include "wsg_50_common/Incr.h"
#include "wsg_50_common/Cmd.h"
#include "wsg_50_common/StateCmd.h"
#include "wsg_50_common/ServerConfiguration.h"

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0
using namespace gripper_command_action;
//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------
//typedef boost::shared_ptr<XamlaSysmonWatch> XamlaSysmonWatchPtr;
std::string prefix;
std::string lastState;
float increment;
bool objectGraspped;

int g_timer_cnt = 0;
ros::Publisher g_pub_state, g_pub_joint, g_pub_moving, heartbeat;

bool g_ismoving = false, g_mode_script = false, g_mode_periodic = false, g_mode_polling = false;
float g_goal_position = NAN, g_goal_speed = NAN, g_speed = 10.0;
bool new_command_msg = false;

SpecializedGripperActionController::Ptr special_action_client_ptr;
GripperActionController::Ptr action_client_ptr;
Gripper_Monitor::Ptr gripper_monitor_ptr;
//XamlaSysmonWatchPtr watcher;

std::mutex g_control_mutex;

bool g_is_homing_active = false;
bool g_is_grasp_active = false;
bool g_is_release_active = false;
bool g_is_stop_active = false;
bool g_is_ack_active = false;

// Initialize 2D array

//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

bool moveSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
    std::unique_lock<std::mutex> lock(g_control_mutex);
    ROS_INFO("Enter move service.");
    if (!g_ismoving)
    {

        if ((req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0))
        {
            ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
            res.error = move(req.width, req.speed, false, true);
        }
        else if (req.width < 0.0 || req.width > 110.0)
        {
            ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] requested: %f", req.width);
            res.error = 255;

            return false;
        }
        else
        {
            ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
            res.error = move(req.width, req.speed, false, true);
        }
        lock.unlock();
        ROS_INFO("Target position reached.");
        g_ismoving = true;
        ros::Duration dt(0.1);
        while (g_ismoving)
            dt.sleep();
    }
    else
    {
        ROS_ERROR("Imposible to move to this position. Controller is not free. ");
        res.error = 255;
        return false;
    }

    return true;
}

bool graspSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
    std::unique_lock<std::mutex> lock(g_control_mutex);
    if (!g_is_grasp_active)
    {

        if ((req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0))
        {
            ROS_INFO("Grasping object at %f with %f mm/s.", req.width, req.speed);
            res.error = grasp(req.width, req.speed, true);
        }
        else if (req.width < 0.0 || req.width > 110.0)
        {
            ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
            res.error = 255;

            return false;
        }
        else
        {
            ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
            res.error = grasp(req.width, req.speed, true);
        }
        g_is_grasp_active = true;
        ros::Duration dt(0.1);
        lock.unlock();
        while (g_is_grasp_active)
            dt.sleep();
    }
    else
    {
        ROS_ERROR("Imposible to move to this position. Controller is not free. ");
        res.error = 255;
        return false;
    }

    ROS_INFO("Object grasped correctly.");
    objectGraspped = true;
    return true;
}

bool incrementSrv(wsg_50_common::Incr::Request &req, wsg_50_common::Incr::Response &res)
{
    std::unique_lock<std::mutex> lock(g_control_mutex);
    if (req.direction == "open")
    {

        if (!objectGraspped)
        {

            float currentWidth = getOpening();
            float nextWidth = currentWidth + req.increment;
            if ((currentWidth < GRIPPER_MAX_OPEN) && nextWidth < GRIPPER_MAX_OPEN)
            {
                //grasp(nextWidth, 1);
                move(nextWidth, 20, true);
                currentWidth = nextWidth;
            }
            else if (nextWidth >= GRIPPER_MAX_OPEN)
            {
                //grasp(GRIPPER_MAX_OPEN, 1);
                move(GRIPPER_MAX_OPEN, 1, true);
                currentWidth = GRIPPER_MAX_OPEN;
            }
        }
        else
        {
            ROS_INFO("Releasing object...");
            release(GRIPPER_MAX_OPEN, 20);
            objectGraspped = false;
        }
    }
    else if (req.direction == "close")
    {

        if (!objectGraspped)
        {

            float currentWidth = getOpening();
            float nextWidth = currentWidth - req.increment;

            if ((currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN)
            {
                //grasp(nextWidth, 1);
                move(nextWidth, 20, true);
                currentWidth = nextWidth;
            }
            else if (nextWidth <= GRIPPER_MIN_OPEN)
            {
                //grasp(GRIPPER_MIN_OPEN, 1);
                move(GRIPPER_MIN_OPEN, 1, true);
                currentWidth = GRIPPER_MIN_OPEN;
            }
        }
    }

    return true;
}

bool releaseSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
    std::unique_lock<std::mutex> lock(g_control_mutex);
    if (!g_is_release_active)
    {

        if ((req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0))
        {
            ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
            res.error = release(req.width, req.speed, true);
            g_is_release_active = true;
        }
        else if (req.width < 0.0 || req.width > 110.0)
        {
            ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
            res.error = 255;

            return false;
        }
        else
        {
            ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
            res.error = release(req.width, req.speed, true);
        }
        lock.unlock();
        ros::Duration dt(0.1);
        while (g_is_release_active)
            dt.sleep();
        ROS_INFO("Object released correctly.");
    }
    else
    {
        ROS_ERROR("Imposible to move to this position. Controller is not free. ");
        res.error = 255;
        return false;
    }

    return true;
}

bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
    std::unique_lock<std::mutex> lock(g_control_mutex);
    if (!g_is_homing_active)
    {

        ROS_INFO("Homing...");
        homing(true);
        g_is_homing_active = true;
        lock.unlock();
        ros::Duration dt(0.1);
        while (g_is_homing_active)
        {
            ROS_INFO("waiting");
            dt.sleep();
        }

        ROS_INFO("Home position reached.");
    }
    else
    {
        ROS_ERROR("Imposible to move to this position. Controller is not free. ");
        return false;
    }
    return true;
}

bool stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
    std::unique_lock<std::mutex> lock(g_control_mutex);
    if (!g_is_stop_active)
    {

        ROS_INFO("Stopping...");
        stop(true);
        g_is_stop_active = true;
        lock.unlock();
        ros::Duration dt(0.1);
        while (g_is_stop_active)
            dt.sleep();

        ROS_INFO("Stopped.");
    }
    else
    {
        ROS_ERROR("Imposible to move to this position. Controller is not free. ");
        return false;
    }

    return true;
}

bool setAccSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
    setAcceleration(req.val);
    return true;
}

bool setForceSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
    setGraspingForceLimit(req.val);
    return true;
}

bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
    std::unique_lock<std::mutex> lock(g_control_mutex);
    if (!g_is_ack_active)
    {
        ack_fault(true);
        g_is_ack_active = true;
        ros::Duration dt(0.1);
        lock.unlock();
        while (g_is_ack_active)
            dt.sleep();

        ROS_INFO("Service Acknowledged.");
    }
    else
    {
        ROS_ERROR("Imposible to move to this position. Controller is not free. ");
        return false;
    }
    return true;
}

/** \brief Callback for goal_position topic (in appropriate modes) */
void position_cb(const wsg_50_common::Cmd::ConstPtr &msg)
{
    ROS_INFO("got position command");
    g_speed = msg->speed;
    g_goal_position = msg->pos;
    // timer_cb() will send command to gripper

    if (g_mode_periodic)
    {
        // Send command to gripper without waiting for a response
        // read_thread() handles responses
        // read/write may be simultaneous, therefore no mutex
        //stop(true);
        if (move(g_goal_position, g_speed, false, true) != 0)
            ROS_ERROR("Failed to send MOVE command");
    }
}

/** \brief Callback for goal_position topic (in appropriate modes) */
void command_cb(const wsg_50_common::StateCmd::ConstPtr &msg)
{
    ROS_INFO("got position command");
    g_speed = msg->speed * 1000;
    g_goal_position = msg->width * 1000;
    double max_force = msg->max_effort;
    // timer_cb() will send command to gripper
    std::unique_lock<std::mutex> lock(g_control_mutex);
    if (g_mode_periodic)
    {
        // Send command to gripper without waiting for a response
        // read_thread() handles responses
        // read/write may be simultaneous, therefore no mutex
        //stop(true);

        if (msg->command == msg->GRASP)
        {
            if (max_force > 0)
                setGraspingForceLimit(max_force, true);

            if (grasp(g_goal_position, g_speed, true) != 0)
            {
                ROS_ERROR("Failed to send GRASP command");
            }
            else
            {
                g_is_grasp_active = true;
            }
        }
        else if (msg->command == msg->RELEASE)
        {
            if (release(g_goal_position, g_speed, true) != 0)
            {
                ROS_ERROR("Failed to send RELEASE command");
            }
            else
            {
                g_is_release_active = true;
            }
        }
        else if (msg->command == msg->MOVE)
        {
            if (move(g_goal_position, g_speed, false, true) != 0)
            {
                ROS_ERROR("Failed to send MOVE command");
            }
            else
            {
                g_ismoving = true;
            }
        }
        else if (msg->command == msg->HOMING)
        {
            if (homing(true) != 0)
            {
                ROS_ERROR("Failed to send HOMING command");
            }
            else
            {
                g_is_homing_active = true;
            }
        }
    }
}

/** \brief Callback for goal_speed topic (in appropriate modes) */
void speed_cb(const std_msgs::Float32::ConstPtr &msg)
{
    g_goal_speed = msg->data;
    g_speed = msg->data;
    // timer_cb() will send command to gripper
}

/** \brief Loop for state polling in modes script and polling. Also sends command in script mode. */
void timer_cb(const ros::TimerEvent &ev)
{
    // ==== Get state values by built-in commands ====

    gripper_monitor_ptr->update();
    /*
    if (g_mode_polling)
    {
        ros::Duration dt(0.1);

        if (!std::isnan(g_goal_position))
        {
            ROS_INFO("send command");
            if (g_control_mutex.lock())
            {
                ROS_INFO("send position command");
                ROS_INFO("Position command: pos=%5.1f, speed=%5.1f", g_goal_position, g_speed);
                float currentWidth = gripper_monitor_ptr->get_last_width();
                float nextWidth = g_goal_position;

                if (currentWidth - nextWidth > 0)
                {
                    if ((currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN)
                    {
                        //grasp(nextWidth, 1);
                        move(nextWidth, 20, false);
                        currentWidth = nextWidth;
                    }
                    else if (nextWidth <= GRIPPER_MIN_OPEN)
                    {
                        //grasp(GRIPPER_MIN_OPEN, 1);
                        move(GRIPPER_MIN_OPEN, 3, false);
                        currentWidth = GRIPPER_MIN_OPEN;
                    }
                }
                else
                {
                    if ((currentWidth < GRIPPER_MAX_OPEN) && nextWidth < GRIPPER_MAX_OPEN)
                    {
                        //grasp(nextWidth, 1);
                        move(nextWidth, 20, false);
                        currentWidth = nextWidth;
                    }
                    else if (nextWidth >= GRIPPER_MAX_OPEN)
                    {
                        //grasp(GRIPPER_MAX_OPEN, 1);
                        move(GRIPPER_MAX_OPEN, 3, false);
                        currentWidth = GRIPPER_MAX_OPEN;
                    }
                }
                g_control_mutex.unlock();
            }
            else
            {
                ROS_WARN("could not lock");
            }
            if (!std::isnan(g_goal_position))
                g_goal_position = NAN;
            if (!std::isnan(g_goal_speed))
                g_goal_speed = NAN;


        }
        action_client_ptr->update(ros::Time::now(), dt);
    }
    else
        return;

    if (gripper_monitor_ptr->isStateReady())
    {
        // ==== Status msg ====
        wsg_50_common::Status status_msg = gripper_monitor_ptr->get_gripper_status_msg();

        g_pub_state.publish(status_msg);


        // ==== Joint state msg ====
        sensor_msgs::JointState joint_states;
        joint_states.header.stamp = ros::Time::now();
        joint_states.header.frame_id = prefix + "_base_link";
        joint_states.name.push_back(prefix + "_base_joint_gripper_left");
        joint_states.name.push_back(prefix + "_base_joint_gripper_right");
        joint_states.position.resize(2);

        joint_states.position[0] = -gripper_monitor_ptr->get_last_width() / 2000.0;
        joint_states.position[1] = gripper_monitor_ptr->get_last_width() / 2000.0;
        joint_states.velocity.resize(2);
        joint_states.velocity[0] = gripper_monitor_ptr->get_last_speed() / 1000.0;
        joint_states.velocity[1] = gripper_monitor_ptr->get_last_speed() / 1000.0;
        joint_states.effort.resize(2);
        joint_states.effort[0] = gripper_monitor_ptr->get_last_f_motor();
        joint_states.effort[1] = gripper_monitor_ptr->get_last_f_motor();

        g_pub_joint.publish(joint_states);
        //graspingState(true);
    }
    */
    // printf("Timer, last duration: %6.1f\n", ev.profile.last_duration.toSec() * 1000.0);
}

/** \brief Reads gripper responses in auto_update mode. The gripper pushes state messages in regular intervals. */
void read_thread(int interval_ms)
{
    ROS_INFO("Thread started");
    ros::Duration dt(0.1);
    status_t status;
    int res;
    bool pub_state = false;

    double rate_exp = 1000.0 / (double)interval_ms;
    std::string names[3] = {"opening", "speed", "force"};

    // Prepare messages
    wsg_50_common::Status status_msg;
    status_msg.status = "UNKNOWN";

    sensor_msgs::JointState joint_states;
    joint_states.header.frame_id = prefix + "_base_link";
    joint_states.name.push_back(prefix + "_base_joint_gripper_left");
    joint_states.name.push_back(prefix + "_base_joint_gripper_right");
    joint_states.position.resize(2);
    joint_states.velocity.resize(2);
    joint_states.effort.resize(2);

    {
        std::unique_lock<std::mutex> guard(g_control_mutex);
        // Request automatic updates (error checking is done below)
        graspingState(interval_ms);
        getOpening(interval_ms);
        getSpeed(interval_ms);
        getForce(interval_ms);
    }

    msg_t msg;
    msg.id = 0;
    msg.data = 0;
    msg.len = 0;
    int cnt[3] = {0, 0, 0};
    auto time_start = std::chrono::system_clock::now();
    xamla_sysmon_msgs::HeartBeat heartmsg;
    int motion = -1;
    while (g_mode_periodic)
    {

        heartmsg.header.stamp = ros::Time::now();
        heartmsg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::GO);
        heartmsg.details = "";

        //action_client_ptr->update(ros::Time::now(), dt);
        {
            //std::lock_guard<std::mutex> guard(g_control_mutex);
            std::unique_lock<std::mutex> guard(g_control_mutex);

            if (g_is_homing_active)
            {
                gripper_monitor_ptr->set_homing_active();
            }
            else if (g_is_grasp_active)
            {
                gripper_monitor_ptr->set_grasp_active();
            }
            else if (g_is_release_active)
            {
                gripper_monitor_ptr->set_release_active();
            }
            else if (g_is_stop_active)
            {
                gripper_monitor_ptr->set_stop_active();
            }
            else if (g_is_ack_active)
            {
                gripper_monitor_ptr->set_ack_active();
            }
            else if (g_ismoving)
            {
            }
            special_action_client_ptr->update(ros::Time::now(), dt);
            //ROS_INFO("update thread");
            gripper_monitor_ptr->update();
            pub_state = gripper_monitor_ptr->isStateReady();
            motion = gripper_monitor_ptr->isMoving();
            status_msg = gripper_monitor_ptr->get_gripper_status_msg();
            if (status_msg.status_id == status_msg.ERROR)
            {
                heartmsg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR);
                heartmsg.details += "Weiss gripper is in Error state. ";
            }
            int *last_cnt = gripper_monitor_ptr->get_count();
            cnt[0] = last_cnt[0];
            cnt[1] = last_cnt[1];
            cnt[2] = last_cnt[2];
            //locking

            g_is_homing_active = gripper_monitor_ptr->get_homing_state();
            g_is_grasp_active = gripper_monitor_ptr->get_grasp_state();
            g_is_release_active = gripper_monitor_ptr->get_release_state();
            g_is_stop_active = gripper_monitor_ptr->get_stop_state();
            g_is_ack_active = gripper_monitor_ptr->get_ack_state();
            // ***** PUBLISH motion message
        }
        if (motion == 0 || motion == 1)
        {
            std_msgs::Bool moving_msg;
            moving_msg.data = motion;
            g_ismoving = motion > 0;
            g_pub_moving.publish(moving_msg);
        }
        // ***** PUBLISH state message & joint message
        if (pub_state)
        {
            //ROS_ERROR_STREAM("joint_states msg: " << joint_states);
            pub_state = false;
            g_pub_state.publish(status_msg);

            joint_states.header.stamp = ros::Time::now();
            joint_states.position[0] = -status_msg.width / 2000.0;
            joint_states.position[1] = status_msg.width / 2000.0;
            joint_states.velocity[0] = status_msg.speed / 1000.0;
            joint_states.velocity[1] = status_msg.speed / 1000.0;
            joint_states.effort[0] = status_msg.force;
            joint_states.effort[1] = status_msg.force;
            g_pub_joint.publish(joint_states);
        }

        // Check # of received messages regularly
        std::chrono::duration<float> t = std::chrono::system_clock::now() - time_start;
        double t_ = t.count();
        if (t_ > 5.0)
        {
            time_start = std::chrono::system_clock::now();
            //printf("Infos for %5.1fHz, %5.1fHz, %5.1fHz\n", (double)cnt[0]/t_, (double)cnt[1]/t_, (double)cnt[2]/t_);

            std::string info = "Rates for ";
            for (int i = 0; i < 3; i++)
            {
                double rate_is = (double)cnt[i] / t_;
                info += names[i] + ": " + std::to_string((int)rate_is) + "Hz, ";
                if (rate_is == 0.0)
                {
                    ROS_ERROR("Did not receive data for %s", names[i].c_str());
                    heartmsg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR);
                    heartmsg.details += "Did not receive data for " + names[i] + " ";
                }
            }
            ROS_DEBUG_STREAM((info + " expected: " + std::to_string((int)rate_exp) + "Hz").c_str());
            cnt[0] = 0;
            cnt[1] = 0;
            cnt[2] = 0;
        }
        heartbeat.publish(heartmsg);
    }

    // Disable automatic updates
    // TODO: The functions will receive an unexpected response
    getOpening(0);
    getSpeed(0);
    getForce(0);
    graspingState(0);

    ROS_INFO("Thread ended");
}

void sigint_handler(int sig)
{
    ROS_INFO("Exiting...");
    g_mode_periodic = false;
    g_mode_script = false;
    g_mode_polling = false;
    ros::shutdown();
}

std::vector<std::string> get_name(XmlRpc::XmlRpcValue controller_list)
{
    std::vector<std::string> result{};
    if (controller_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        std::cout << controller_list << std::endl;
        for (int32_t j = 0; j < controller_list.size(); ++j)
        {
            std::cout << controller_list[j] << std::endl;

            if (controller_list[j].size() == 4)
            {
                if (controller_list[j]["name"].getType() == XmlRpc::XmlRpcValue::TypeString && controller_list[j]["type"].getType() == XmlRpc::XmlRpcValue::TypeString)
                {
                    std::string name = static_cast<std::string>(controller_list[j]["name"]);
                    std::string action_ns = static_cast<std::string>(controller_list[j]["action_ns"]);
                    std::string type = static_cast<std::string>(controller_list[j]["type"]);
                    std::string joint = static_cast<std::string>(controller_list[j]["joints"][0]);
                    std::cout << "Name : " << name << " -> " << type << std::endl;
                    if (type == "WeissGripperCmd")
                    {
                        result.push_back(name);
                        result.push_back(action_ns);
                        result.push_back(type);
                        result.push_back(joint);
                    }
                }
            }
        }
    }
    return result;
}

/**
 * The main function
 */

int main(int argc, char **argv)
{

    ros::init(argc, argv, "wsg_gripper");
    ros::NodeHandle root("/");
    ros::NodeHandle nh("~");
    signal(SIGINT, sigint_handler);

    std::string ip, protocol, com_mode;
    int port, local_port;
    double rate, grasping_force;
    bool use_udp = false;
    lastState = "NONE";

    nh.param("ip", ip, std::string("192.168.1.20"));
    nh.param("port", port, 1000);
    nh.param("local_port", local_port, 1501);
    nh.param("protocol", protocol, std::string("tcp"));
    nh.param("com_mode", com_mode, std::string("auto_update"));
    nh.param("rate", rate, 30.0); // With custom script, up to 30Hz are possible
    nh.param("grasping_force", grasping_force, 0.0);
    nh.param("prefix", prefix, std::string(""));
    XmlRpc::XmlRpcValue tmp_list;
    nh.getParam("controller_list", tmp_list);
    std::vector<std::string> result = get_name(tmp_list);
    if (result.empty())
    {
        ROS_ERROR("Could not find controller_list");
        return 0;
    }
    std::string controller_name = result[0];
    std::string action_ns = result[1];
    if (protocol == "udp")
        use_udp = true;
    else
        protocol = "tcp";
    if (com_mode == "script")
        g_mode_script = true;
    else if (com_mode == "auto_update")
        g_mode_periodic = true;
    else
    {
        com_mode = "polling";
        g_mode_polling = true;
    }
    special_action_client_ptr.reset(new SpecializedGripperActionController(controller_name + "/" + action_ns));
    //action_client_ptr.reset(new GripperActionController(controller_name + "/gripper_cmd"));

    ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", ip.c_str(), port, protocol.c_str(), com_mode.c_str());

    // Connect to device using TCP/USP
    int res_con;
    if (!use_udp)
        res_con = cmd_connect_tcp(ip.c_str(), port);
    else
        res_con = cmd_connect_udp(local_port, ip.c_str(), port);

    if (res_con == 0)
    {
        ROS_INFO("Gripper connection stablished");

        // Services
        ros::ServiceServer infoS, moveSS, graspSS, releaseSS, homingSS, stopSS, ackSS, incrementSS, setAccSS, setForceSS;

        if (g_mode_script || g_mode_polling)
        {

            incrementSS = nh.advertiseService(controller_name + "/move_incrementally", incrementSrv);

            setAccSS = nh.advertiseService(controller_name + "/set_acceleration", setAccSrv);
            setForceSS = nh.advertiseService("set_force", setForceSrv);
        }
        moveSS = nh.advertiseService(controller_name + "/move", moveSrv);
        graspSS = nh.advertiseService(controller_name + "/grasp", graspSrv);
        releaseSS = nh.advertiseService(controller_name + "/release", releaseSrv);
        homingSS = nh.advertiseService(controller_name + "/homing", homingSrv);
        stopSS = nh.advertiseService(controller_name + "/stop", stopSrv);
        ackSS = nh.advertiseService(controller_name + "/ack", ackSrv);

        // Subscriber
        ros::Subscriber sub_command, sub_position, sub_speed;
        if (g_mode_script || g_mode_periodic || g_mode_polling)
        {
            sub_position = nh.subscribe(controller_name + "/goal_position", 5, position_cb);
            sub_command = nh.subscribe(controller_name + "/goal_command", 5, command_cb);
        }
        if (g_mode_script)
            sub_speed = nh.subscribe(controller_name + "/goal_speed", 5, speed_cb);


        //action_client_ptr->init(root, nh);
        // Publisher
        g_pub_state = nh.advertise<wsg_50_common::Status>(controller_name + "/status", 1000);
        g_pub_joint = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        if (g_mode_script || g_mode_periodic)
            g_pub_moving = nh.advertise<std_msgs::Bool>(controller_name + "/moving", 10);

        heartbeat = nh.advertise<xamla_sysmon_msgs::HeartBeat>(controller_name + "/heartbeat", 1);
        xamla_sysmon_msgs::HeartBeat msg;
        msg.header.stamp = ros::Time::now();

        msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::STARTING);
        msg.details = TopicHeartbeatStatus::generateMessageText(TopicHeartbeatStatus::intToStatusCode(msg.status));
        heartbeat.publish(msg);
        //ROS_INFO("Ready to use, homing now...");
        homing(false);

        if (grasping_force > 0.0)
        {
            ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
            setGraspingForceLimit(grasping_force);
        }
        gripper_monitor_ptr.reset(new Gripper_Monitor((int)(1000.0 / rate)));
        special_action_client_ptr->init(controller_name, root, nh);
        //watcher.reset(new XamlaSysmonWatch());
        //watcher->start();
        ROS_INFO("Init done. Starting timer/thread with target rate %.1f.", rate);
        std::thread th;
        ros::Timer tmr;
        if (g_mode_polling || g_mode_script)
            tmr = nh.createTimer(ros::Duration(1.0 / rate), timer_cb);
        if (g_mode_periodic)
            th = std::thread(read_thread, (int)(1000.0 / rate));

        ros::spin();
    }
    else
    {
        ROS_ERROR("Unable to connect, please check the port and address used.");
    }

    ROS_INFO("Exiting...");
    g_mode_periodic = false;
    g_mode_script = false;
    g_mode_polling = false;
    //if (watcher)
    //    watcher->shutdown();
    sleep(1);
    cmd_disconnect();

    return 0;
}

//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
