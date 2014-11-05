#include "SnakeMotorWrapper.h"
SnakeMotorWrapper::SnakeMotorWrapper(std::string port_name)
{
	motor_abs_init(const_cast<char*>(port_name.c_str()));
}
void SnakeMotorWrapper::callback(const geometry_msgs::Twist::ConstPtr& msg)
{

}
