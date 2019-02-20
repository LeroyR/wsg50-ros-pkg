#include "wsg_driver/can/wsg_can.h"

#include <ros/ros.h>

WSGCan::WSGCan(std::string device, uint commandID) : CanDevice(device)
{
  addFilterId(commandID + 1);
}

void WSGCan::init()
{
  reader_ = std::thread(&WSGCan::readValues, this);
  running = true;
}

// void WSGCan::sendFrame(GripperFrame frame) {
//
//}

void WSGCan::move(float pose, float speed)
{
  unsigned char payload[8];

  // Copy target width and speed
  memcpy(&payload[0], &pose, sizeof(float));
  memcpy(&payload[4], &speed, sizeof(float));

  std::vector<unsigned char> data = { 0xAA, 0xAA, 0xAA, COMMAND_MOVE, 0x08, 0 };

  for (auto c : payload)
  {
    data.push_back(c);
  }

  data.push_back(0xFF);
  data.push_back(0xFF);

  std::vector<unsigned char> part1(data.begin(), data.begin() + 8);
  std::vector<unsigned char> part2(data.begin() + 8, data.end());

  write(0x3C, part1);
  write(0x3C, part2);

  // std::vector<unsigned char> data = {0xAA,0xAA,0xAA,0xB1,0x08,0,0,0};
  // std::vector<unsigned char> data2 = {0,0x42,0,0xA0,0x41, 0x00,0xFF,0xFF};
}

void WSGCan::readValues()
{
  while (running)
  {
    auto ret = read();
    // ROS_INFO_STREAM(ret);

    if (ret.data.size() < 6 || ret.data[0] != 0xAA || ret.data[1] != 0xAA || ret.data[2] != 0xAA)
    {
      ROS_ERROR_STREAM("unparseable data" << ret);
      continue;
    }

    GripperFrame msg;
    msg.cmd = ret.data[3];

    auto msg_len = ((unsigned short)ret.data[4] | ((unsigned short)ret.data[5] << 8));
    msg_len += 6;  // header
    msg_len += 2;  // crc

    int nummsg = ((msg_len + msg_len % 8) / 8);
    // ROS_INFO_STREAM("reading msg_len: " << msg_len << " in " << nummsg << " msgs");

    for (int i = 6; i < msg_len; i++)
    {
      if (i % 8 == 0)
      {  // read next frame
        ret = read();
      }
      msg.data.push_back(ret.data[i % 8]);
    }

    for (auto cb : cbs_)
    {
      cb(msg);
    }

    // ROS_INFO_STREAM(msg);
  }
}

WSGCan::~WSGCan()
{
  running = false;
  reader_.join();
}