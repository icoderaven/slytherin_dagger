#pragma once
#include <geometry_msgs/Twist.h>
#include <string.h>
extern "C"{
#include "motor_abs.h"
}

class SnakeMotorWrapper{
private:
public:
	SnakeMotorWrapper(std::string);
	void callback(const geometry_msgs::Twist::ConstPtr& );
};
