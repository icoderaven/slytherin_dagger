#include <ros/ros.h>
#include "SnakeMotorWrapper.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "snake_controller");
  ros::NodeHandle nh;

  SnakeMotorWrapper _wrapper("/dev/ttyUSB0");

  ros::Subscriber sub = nh.subscribe("cmd_vel", 10,&SnakeMotorWrapper::callback, &_wrapper);
  ros::spin();

  return 0;
}
