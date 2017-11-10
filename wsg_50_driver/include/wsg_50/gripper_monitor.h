//======================================================================
/**
 *  @file
 *  interface.h
 *
 *  @section interface.h_general General file information
 *
 *  @brief
 *
 *
 *  @author alemme
 *  @date  10.11.2017
 *
 *
 *  @section cmd_submit.h_copyright Copyright
 *
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *
 *  The distribution of this code and excerpts thereof, neither in
 *  source nor in any binary form, is prohibited, except you have our
 *  explicit and written permission to do so.
 *
 */
//======================================================================

#ifndef GRIPPER_MONITOR_H_
#define GRIPPER_MONITOR_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include "wsg_50/common.h"
#include "wsg_50/msg.h"
#include "wsg_50_common/Status.h"
#include "sensor_msgs/JointState.h"

class Gripper_Monitor
{
  public:
    typedef boost::shared_ptr<Gripper_Monitor> Ptr;
    Gripper_Monitor(int ms_interval);
    int isMoving();
    int update();
    inline bool isStateReady()
    {
        if (this->pub_state)
        {
            this->pub_state = false;
            return true;
        }
        return false;
    }
    inline wsg_50_common::Status get_gripper_status_msg()
    {
        status_msg.acc = this->acc;
        return status_msg;
    }
    void set_homing_active()
    {
        this->active_homing = true;
    }

    bool get_homing_state()
    {
        return this->active_homing;
    }

    void set_grasp_active()
    {
        this->active_grasp = true;
    }

    bool get_grasp_state()
    {
        return this->active_grasp;
    }
    void set_release_active()
    {
        this->active_release = true;
    }

    bool get_release_state()
    {
        return this->active_release;
    }

    void set_stop_active()
    {
        this->active_stop = true;
    }

    bool get_stop_state()
    {
        return this->active_stop;
    }

    void set_ack_active()
    {
        this->active_ack = true;
    }

    bool get_ack_state()
    {
        return this->active_ack;
    }


    float get_last_width();
    float get_last_speed();
    float get_last_f_motor();
    int *get_count()
    {
        return this->cnt;
    }

  protected:
    status_t status;
    gripstatus_t grip_status;
    std::string last_grip_state_text;
    int res;
    bool pub_state = false;

    std::string names[3] = {"opening", "speed", "force"};

    // Prepare messages
    wsg_50_common::Status status_msg;
    /*
    status_msg.status = info.state_text;
    status_msg.status_id = info.state;
    status_msg.width = info.position;
    status_msg.speed = info.speed;
    status_msg.acc = acc;
    status_msg.force = info.f_motor;
    status_msg.force_finger0 = info.f_finger0;
    status_msg.force_finger1 = info.f_finger1;
    */
    sensor_msgs::JointState joint_states;
    int motion = -1;
    msg_t msg;
    int cnt[3] = {0, 0, 0};

    float grasp_force_limit;
    float acc;
    float opening_speed_force;
    bool active_homing = false;
    bool active_grasp = false;
    bool active_stop = false;
    bool active_ack = false;
    bool active_release = false;
};

#endif /* GRIPPER_MONITOR_H_ */
