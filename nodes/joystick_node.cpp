#include <string>
#include <ros/ros.h>
//#include <joy/Joy.h> No longer required
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

// --------------------------------------
class TeleopJoystick
{
public:
  TeleopJoystick();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int forward_backward_, left_right_, up_down_, rotate_, init_, pause_, retract_, execute_, homing_, stepped_;
  double translation_scale_, angular_scale_;
  ros::Publisher vel_pub_, init_pub_, pause_pub_, retract_pub_, execute_pub_, homing_pub_, stepped_pub_;
  ros::Subscriber joy_sub_;
  std::string output_name_;
};

// --------------------------------------
TeleopJoystick::TeleopJoystick():
  forward_backward_(1),
  left_right_(0),
  up_down_(5),
  rotate_(4),
  init_(1),
  pause_(0),
  retract_(11),
  execute_(10),
  homing_(9),
  stepped_(8),
  translation_scale_(0.1),
  angular_scale_(1.0)
{
  ros::NodeHandle nh_forname("~");
  nh_forname.getParam("forward_backward", forward_backward_);
  nh_forname.getParam("left_right", left_right_);
  nh_forname.getParam("up_down", up_down_);
  nh_forname.getParam("rotate", rotate_);
  nh_forname.getParam("init", init_);
  nh_forname.getParam("pause", pause_);
  nh_forname.getParam("retract", retract_);
  nh_forname.getParam("execute", execute_);
  nh_forname.getParam("homing", homing_);
  nh_forname.getParam("translation_scale", translation_scale_);
  nh_forname.getParam("angular_scale", angular_scale_);
  nh_forname.getParam("output_name", output_name_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(output_name_, 1);
  init_pub_ = nh_.advertise<std_msgs::Empty>("/initialize", 1);
  pause_pub_ = nh_.advertise<std_msgs::Empty>("/pause", 1);
  retract_pub_ = nh_.advertise<std_msgs::Empty>("/retract", 1);
  execute_pub_ = nh_.advertise<std_msgs::Empty>("/execute", 1);
  homing_pub_ = nh_.advertise<std_msgs::Empty>("/homing", 1);
  stepped_pub_ = nh_.advertise<std_msgs::Empty>("/stepped", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoystick::joyCallback, this);
}

// --------------------------------------
void TeleopJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;

 
  // Translation commands
  twist.linear.x = translation_scale_*joy->axes[forward_backward_];
  twist.linear.y = translation_scale_*joy->axes[left_right_];
  twist.linear.z = translation_scale_*joy->axes[up_down_];;
      
  // Rotation commands
  // The x and y angular rates are ignored as they are 
  // preserved for the linear motion
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_scale_*joy->axes[rotate_];
        
  vel_pub_.publish(twist);

  // Takeoff, land and reset commands
  if(joy->buttons[init_])
    {
      std::cout<<"[Joystick Driver] Initialize!\n";
      std_msgs::Empty empty;
      init_pub_.publish(empty);
    }

  if(joy->buttons[pause_])
    {
      std::cout<<"[Joystick Driver] Pause!\n";
      std_msgs::Empty empty;
      pause_pub_.publish(empty);
    }

  if(joy->buttons[retract_])
    {
      std::cout<<"[Joystick Driver] Retract!\n";
      std_msgs::Empty empty;
      retract_pub_.publish(empty);
    }

  if(joy->buttons[execute_])
    {
      std::cout<<"[Joystick Driver] Execute!\n";
      std_msgs::Empty empty;
      execute_pub_.publish(empty);
    }

  if(joy->buttons[homing_])
    {
      std::cout<<"[Joystick Driver] Home!\n";
      std_msgs::Empty empty;
      homing_pub_.publish(empty);
    }
  if(joy->buttons[stepped_])
    {
      std::cout<<"[Joystick Driver] Stepped!\n";
      std_msgs::Empty empty;
      stepped_pub_.publish(empty);
    }
}

// --------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_node");
  TeleopJoystick teleop_joystick;

  ros::spin();
}

