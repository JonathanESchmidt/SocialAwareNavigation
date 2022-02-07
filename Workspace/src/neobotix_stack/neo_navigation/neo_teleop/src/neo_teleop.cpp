/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class TeleopNeo
{
public:
  TeleopNeo();
  ros::NodeHandle nh_;
  void sendCmd();
  double l_scale_x, l_scale_y, a_scale_z;
  double joy_command_x;
  double joy_command_y;
  double joy_command_z;
  double last_joy_commands_x[20];
  double last_joy_commands_y[20]; 
  double last_joy_commands_z[20];
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  bool smooth;
  int linear_x, linear_y, angular_z;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  geometry_msgs::Twist vel;
  ros::Time lastCmd;
  ros::Duration timeOut;
  bool activeJoy;
  bool deadman;
  int deadman_button;
};


TeleopNeo::TeleopNeo():
  linear_x(1), linear_y(0), angular_z(2)
{
  activeJoy = false;
  deadman = false;
  smooth = true;
  lastCmd = ros::Time(0);
  double timeout;
  joy_command_x = 0;
  joy_command_y = 0;
  joy_command_z = 0;
  for(int i = 0; i < 20; i++)
  {
	last_joy_commands_x[i] = 0;
	last_joy_commands_y[i] = 0;
	last_joy_commands_z[i] = 0;
  }
  if(nh_.hasParam("smooth"))
  {
  	nh_.getParam("smooth", smooth);
	ROS_INFO("smooth: %i", smooth);
  }
  else
  {
  	ROS_INFO("smooth nicht gesetzt, verwende Standard: True");
  	smooth = true;
  }
  if(nh_.hasParam("timeOut"))
  {
  	nh_.getParam("timeOut", timeout);
  }
  else
  {
  	ROS_INFO("kein TimeOut gesetzt, verwende Standard");
  	timeout = 1.0;
  }
  if(nh_.hasParam("deadman_button"))
  {
  	nh_.getParam("deadman_button", deadman_button);
  }
  else
  {
  	ROS_INFO("kein deadmanbutton gesetzt, verwende Standard");
  	deadman_button = 5;
  }
  ROS_INFO("TimeOut: %f ", timeout);
  timeOut = ros::Duration(timeout);
  nh_.param("axis_linear_x", linear_x, linear_x);
  nh_.param("scale_linear_x", l_scale_x, l_scale_x);
  nh_.param("axis_linear_y", linear_y, linear_y);
  nh_.param("scale_linear_y", l_scale_y, l_scale_y);
  nh_.param("axis_angular_z", angular_z, angular_z);
  nh_.param("scale_angular_z", a_scale_z, a_scale_z);
 
  ROS_INFO("DeadmanButton: %i ", deadman_button);

  ROS_INFO("started joystick drive with ");

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopNeo::joyCallback, this);

}

void TeleopNeo::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  activeJoy = true;
  deadman = (bool)joy->buttons[deadman_button];
  lastCmd = ros::Time::now();
  joy_command_z = a_scale_z*joy->axes[angular_z];
  joy_command_x = l_scale_x*joy->axes[linear_x];
  joy_command_y = l_scale_y*joy->axes[linear_y];
  
}

void TeleopNeo::sendCmd()
{
  if(deadman == true)
  {
    if(smooth == true)
    {
	    //Array neu anordnen -- Rearranging the array
	    for(int i = 18; i >= 0; i--)
	    {
		last_joy_commands_x[i+1] = last_joy_commands_x[i];
		last_joy_commands_y[i+1] = last_joy_commands_y[i];
		last_joy_commands_z[i+1] = last_joy_commands_z[i];
	    }
	    //neuen wert in position 0 schreiben -- Set new value in position 0
	    last_joy_commands_x[0] = joy_command_x;
	    last_joy_commands_y[0] = joy_command_y;
	    last_joy_commands_z[0] = joy_command_z;
	    //mittelwert bilden -- Finding mean value 
	    vel.angular.z = 0;
	    vel.linear.x = 0;
	    vel.linear.y = 0;
	    for(int g = 0; g < 20; g++)
	    {
		vel.angular.z += last_joy_commands_z[g];
		vel.linear.x += last_joy_commands_x[g];
		vel.linear.y += last_joy_commands_y[g];
	    }
	    vel.angular.z = (vel.angular.z/20);
	    vel.linear.x = (vel.linear.x/20);
	    vel.linear.y = (vel.linear.y/20);
    }
    else
    {
	vel.angular.z = joy_command_z;
	vel.linear.x = joy_command_x;
	vel.linear.y = joy_command_y;
    }
    vel_pub_.publish(vel);
  }
  else
  {
    if(activeJoy)
    {
      vel.angular.z = 0;
      vel.linear.x = 0;
      vel.linear.y = 0;
      vel_pub_.publish(vel);
      activeJoy = false;
      smooth = false;
    }
  }
};


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "teleop_Neo");
  TeleopNeo teleop_Neo;
  ros::Rate loop_rate(50); // Hz
  while(teleop_Neo.nh_.ok())
  {
    teleop_Neo.sendCmd();
    loop_rate.sleep();
    ros::spinOnce();
  }
}
