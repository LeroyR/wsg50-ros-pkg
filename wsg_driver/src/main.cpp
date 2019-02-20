#include <ros/ros.h>

#include <wsg_driver/can/wsg_can.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wsg_driver");
  ros::NodeHandle nh("~");

  WSGCan can0("can0", 0x3C);
  can0.init();

  can0.move(0, 20);

  can0.attach([&](GripperFrame frame) {
    //
    if (frame.cmd == GripperData::COMMAND_ID)
    {
      auto data = GripperData::fromFrame(frame);
      ROS_INFO_STREAM(data);
      // ROS_INFO_STREAM(frame);
      if (data.pose > 30)
      {
        can0.move(0, 20);
      }
      else if (data.pose < 2)
      {
        can0.move(32, 20);
      }
      else if (data.is_stopped())
      {
        can0.move(32, 20);
      }
    }
    else
    {
      ROS_INFO_STREAM(frame);
    }

  });

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}