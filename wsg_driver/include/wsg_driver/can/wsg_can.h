#pragma once

#include <thread>
#include <functional>

#include "wsg_driver/can/can_device.h"
#include "wsg_driver/gripper_data.h"

class WSGCan : public CanDevice
{
  using Callback = std::function<void(GripperFrame)>;
  static const unsigned char COMMAND_MOVE = 0xB1;

public:
  WSGCan(std::string device, uint commandID);
  ~WSGCan();

  void attach(Callback c)
  {
    cbs_.push_back(c);
  }
  void move(float pose, float speed);

  void init();

private:
  void readValues();

  std::thread reader_;
  bool running{ false };

  std::vector<Callback> cbs_;
};
