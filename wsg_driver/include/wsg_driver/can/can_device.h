#pragma once

#include <iostream>

#include <string>
#include <vector>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

struct CanFrame
{
  uint id;
  std::vector<unsigned char> data;
};

std::ostream& operator<<(std::ostream& os, const CanFrame& c);

class CanDevice
{
public:
  CanDevice(std::string device);
  ~CanDevice();
  void write(uint id, std::vector<unsigned char> data);
  void addFilterId(uint id);
  void write(CanFrame frame);
  CanFrame read();

private:
  int socket_;
  struct sockaddr_can addr;
  struct ifreq ifr;
};