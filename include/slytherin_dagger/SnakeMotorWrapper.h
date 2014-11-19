#pragma once
#include <geometry_msgs/Twist.h>

class SnakeMotorWrapper{
private:
	int a;
public:
	SnakeMotorWrapper(){a = 0;}

	void callback(const geometry_msgs::Twist::ConstPtr& );
};
