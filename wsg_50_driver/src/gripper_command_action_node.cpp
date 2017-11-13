#include "wsg_50/include/gripper_command_action.h"

using gripper_command_action::GripperActionController;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "gripper_command_action");
  GripperActionController action;
  action.run();

  return 0;
}