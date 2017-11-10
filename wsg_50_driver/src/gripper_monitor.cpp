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
 *  @date	10.11.2017
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

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------
#include <ros/ros.h>
#include "wsg_50/gripper_monitor.h"
#include "wsg_50/cmd.h"
#include "wsg_50/functions.h"

Gripper_Monitor::Gripper_Monitor(int ms_interval)
{
    pub_state = false;

    // Prepare messages
    status_msg.status = "UNKNOWN";
    status_msg.speed = 0;
    status_msg.force = 0;
    status_msg.width = 0;
    this->last_grip_state_text = "UNKNOWN";

    // Request automatic updates (error checking is done below)
    //getOpening(ms_interval);
    //getSpeed(ms_interval);
    //getForce(ms_interval);

    msg.id = 0;
    msg.data = 0;
    msg.len = 0;
}

int Gripper_Monitor::isMoving()
{
    return motion;
}

float Gripper_Monitor::get_last_width()
{
    return status_msg.width;
}

float Gripper_Monitor::get_last_speed()
{
    return status_msg.speed;
}

float Gripper_Monitor::get_last_f_motor()
{
    return status_msg.force;
}

int Gripper_Monitor::update()
{
    // Receive gripper response
    if (msg.len > 0)
        msg_free(&msg);

    res = msg_receive(&msg);
    if (res < 0 || msg.len < 2)
    {
        ROS_ERROR("Gripper response failure: too short");
        return -1;
    }

    float val = 0.0;
    status = cmd_get_response_status(msg.data);

    // Decode float for opening/speed/force
    if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6)
    {
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Gripper response failure for opening/speed/force\n");
            return -1;
        }
        val = convert(&msg.data[2]);
    }

    // Handle response types
    motion = -1;
    switch (msg.id)
    {
    /*** Opening ***/
    case 0x43:
        status_msg.width = val;
        pub_state = true;
        cnt[0]++;
        break;

    /*** Speed ***/
    case 0x44:
        status_msg.speed = val;
        cnt[1]++;
        break;

    /*** Force ***/
    case 0x45:
        status_msg.force = val;
        cnt[2]++;
        break;

    /*** Move ***/
    // Move commands are sent from outside this thread
    case 0x21:
        if (status == E_SUCCESS)
        {
            ROS_INFO("Position reached");
            motion = 0;
        }
        else if (status == E_AXIS_BLOCKED)
        {
            ROS_INFO("Axis blocked");
            motion = 0;
        }
        else if (status == E_CMD_PENDING)
        {
            ROS_INFO("Movement started");
            motion = 1;
        }
        else if (status == E_ALREADY_RUNNING)
        {
            ROS_INFO("Movement error: already running");
        }
        else if (status == E_CMD_ABORTED)
        {
            ROS_INFO("Movement aborted");
            motion = 0;
        }
        else
        {
            ROS_INFO("Movement error");
            motion = 0;
        }
        break;

    /*** Stop ***/
    // Stop commands are sent from outside this thread
    case 0x22:
        // Stop command; nothing to do
        if (status == E_SUCCESS)
        {
            ROS_INFO("Command STOP successful\n");
            this->active_stop = false;
        }else if(status == E_CMD_PENDING){
            ROS_INFO("Command STOP PENDING\n");
            this->active_stop = true;
        }else{
            this->active_stop = false;
            ROS_ERROR("Command STOP not successful\n");
            return -1;
        }
        break;
    case 0x20:
        //homing
        if (status == E_SUCCESS)
        {
            ROS_INFO("Command HOMING successful\n");
            this->active_homing = false;
        }else if(status == E_CMD_PENDING){
            ROS_INFO("Command HOMING PENDING\n");
            this->active_homing = true;
        }else{
            this->active_homing = false;
            ROS_ERROR("Command HOMING not successful\n");
            return -1;
        }
        break;
    case 0x24:
    { //ack_fault
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Command ACK not successful\n");
            return -1;
        }
        this->active_ack = false;
        break;
    }
    case 0x25:
    { //grasp

        if (status == E_SUCCESS)
        {
            ROS_INFO("Command GRASP successful\n");
            this->active_grasp = false;
        }else if(status == E_CMD_PENDING){
            ROS_INFO("Command GRASP PENDING\n");
            this->active_grasp = true;
        }else{
            this->active_grasp = false;
            ROS_ERROR("Command GRASP not successful\n");
            return -1;
        }
        break;
    }
    case 0x26:
    { //RELEASE
        if (status == E_SUCCESS)
        {
            ROS_INFO("Command GRASP successful\n");
            this->active_release = false;
        }else if(status == E_CMD_PENDING){
            ROS_INFO("Command GRASP PENDING\n");
            this->active_release = true;
        }else{
            this->active_release = false;
            ROS_ERROR("Command GRASP not successful\n");
            return -1;
        }
        break;
    }
    case 0x30:
    { //SET ACCELERATIONs
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Command SET ACCELERATION not successful\n");
            return -1;
        }
        break;
    }
    case 0x31:
    { //GET ACCELERATIONs
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Command GET ACCELERATION not successful\n");
            return -1;
        }
        unsigned char vResult_[4];
        vResult_[0] = msg.data[2];
        vResult_[1] = msg.data[3];
        vResult_[2] = msg.data[4];
        vResult_[3] = msg.data[5];
        this->acc = convert(vResult_);
        break;
    }
    case 0x32:
    { //SET GRASPING FORCE LIMIT
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Command SET GRASPING FORCE LIMIT not successful\n");
            return -1;
        }
        break;
    }
    case 0x33:
    { //getGraspingForceLimit
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Command GET GRASPING FORCE not successful\n");
            return -1;
        }
        unsigned char vResult[4];
        vResult[0] = msg.data[2];
        vResult[1] = msg.data[3];
        vResult[2] = msg.data[4];
        vResult[3] = msg.data[5];
        this->grasp_force_limit = convert(vResult);
        break;
    }
    case 0x40:
    {
        //systemState
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Command systemState not successful\n");
            return -1;
        }
        getStateValues(msg.data);

        break;
    }
    case 0x41:
    {
        //graspingState
        if (status != E_SUCCESS)
        {
            ROS_ERROR("Command SET GRASPING STATE not successful\n");
            return -1;
        }
        this->grip_status = (gripstatus_t)msg.data[2];
        const char *state = grip_status_to_str(this->grip_status);
        if (state)
        {
            this->status_msg.status = std::string(state);
            this->status_msg.status_id = this->grip_status;
        }
        break;
    }
    default:
        ROS_INFO("Received unknown respone 0x%02x (%2dB)\n", msg.id, msg.len);
    }

    return 1;
}
