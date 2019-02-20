#include "wsg_driver/can/can_device.h"

#include <iostream>

#include <stdio.h>
#include <string.h>

#include <unistd.h>

#include <ros/ros.h>

std::ostream& operator<<(std::ostream& os, const CanFrame& c)
{
  os << std::setfill('0') << std::setw(3) << std::hex << std::uppercase << (int)c.id;
  os << " [" << c.data.size() << "] ";
  for (const auto& v : c.data)
  {
    os << std::setfill('0') << std::setw(2) << std::hex << std::uppercase << (int)v << " ";
  }
  return os;
}

CanDevice::CanDevice(std::string device)
{
  if ((socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    ROS_ERROR_STREAM("Error while opening socket");
  }

  ::strcpy(ifr.ifr_name, device.c_str());
  ::ioctl(socket_, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    ROS_ERROR_STREAM("Error in socket bind");
  }

  struct can_frame frame;

  frame.can_id = 0xFF;

  frame.can_dlc = 0;

  int nbytes = ::write(socket_, &frame, sizeof(struct can_frame));
}

CanDevice::~CanDevice()
{
  std::cout << "closing socket" << std::endl;
  ::close(socket_);
}

void CanDevice::write(uint id, std::vector<unsigned char> data)
{
  assert(data.size() <= 8);

  struct can_frame frame;

  frame.can_id = id;

  frame.can_dlc = data.size();
  for (unsigned int i = 0; i < data.size(); i++)
  {
    frame.data[i] = data[i];
  }

  int nbytes = ::write(socket_, &frame, sizeof(struct can_frame));

  return;
}

void CanDevice::write(CanFrame frame)
{
  write(frame.id, frame.data);
}

void CanDevice::addFilterId(uint id)
{
  struct can_filter filter;
  filter.can_id = id;
  filter.can_mask = CAN_SFF_MASK;

  if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
  {
    ROS_ERROR_STREAM("Error in filter");
  }
}

CanFrame CanDevice::read()
{
  struct can_frame cframe;
  CanFrame frame;

  // Read in a CAN frame
  int numBytes = ::read(socket_, &cframe, CAN_MTU);

  frame.id = cframe.can_id;

  for (int i = 0; i < (int)cframe.can_dlc; i++)
  {
    frame.data.push_back(cframe.data[i]);
  }

  return frame;
}
