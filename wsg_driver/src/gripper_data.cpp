#include "wsg_driver/gripper_data.h"

#include <iomanip>


float convert(std::vector<unsigned char> d, unsigned int i) {
    float tmp;
    unsigned int src = d[i+3] * 16777216 + d[i+2] * 65536 + d[i+1] * 256 + d[i];
    memcpy(&tmp, &src, sizeof tmp);

    return tmp;
}


GripperData GripperData::fromFrame(GripperFrame frame) {
    GripperData data;

    if(frame.cmd != COMMAND_ID) throw "error";
    if(frame.data.size() != 25) throw "error";

    data.state = frame.data[2];

    data.pose = convert(frame.data, 3);
    data.speed = convert(frame.data, 7);
    data.force = convert(frame.data, 11);
    data.force_l = convert(frame.data, 15);
    data.force_r = convert(frame.data, 19);

    return data;
}

std::ostream& operator<<(std::ostream& os, const GripperData& g) {
  os << "Gripper: " << g.state << std::dec;
  os << " | pose: " << g.pose;
  os << " | speed: " << g.speed;
  os << " | force: " << g.force << "(" << g.force_l << "|" << g.force_r << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const GripperFrame& c) {
  os << "GripperFrame:" << std::setfill('0') << std::setw(3) << std::hex << std::uppercase << (int) c.cmd;
  os << " [" << std::dec << c.data.size() << "] ";
  os << std::setfill('0') << std::setw(2) << std::hex << std::uppercase;
  for (const auto& v : c.data) {
    os << std::setw(2) << (int) v << " ";
  }
  return os;
}