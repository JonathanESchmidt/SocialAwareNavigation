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


#include <neo_PlatformCtrl/DiffDrive2WKinematics.h>
#include <geometry_msgs/Vector3.h>

#define DIFF_PI 3.14159265

DiffDrive2WKinematics::DiffDrive2WKinematics()
{
	m_dAxisLength = 0;
}

void DiffDrive2WKinematics::execForwKin(const sensor_msgs::JointState& js, nav_msgs::Odometry& odom, OdomPose& cpose)
{
	current_time = ros::Time::now();
	//velocities:
	odom.twist.twist.linear.x = 0.5 * (js.velocity[0] +  js.velocity[1]) * m_dDiam * 0.5;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = (js.velocity[0] -  js.velocity[1]) * m_dDiam /2  / m_dAxisLength;
	//positions:
	double dt = (current_time - last_time).toSec();
	cpose.xAbs += odom.twist.twist.linear.x * dt * cos(cpose.phiAbs);
	cpose.yAbs += odom.twist.twist.linear.x *dt * sin(cpose.phiAbs);
	cpose.phiAbs += odom.twist.twist.angular.z * dt;
	odom.pose.pose.position.x = cpose.xAbs;
	odom.pose.pose.position.y = cpose.yAbs;
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(cpose.phiAbs);
	last_time = current_time;
}

void DiffDrive2WKinematics::execInvKin(const geometry_msgs::Twist& twist, trajectory_msgs::JointTrajectory& traj)
{
	traj.joint_names.clear();
	traj.points.clear();

	//angular velocity in rad
	trajectory_msgs::JointTrajectoryPoint point;
	point.velocities.resize(4);
	// w1:
	traj.joint_names.push_back("wheel_front_left_base_link");
	point.velocities[0] = (twist.linear.x + (twist.angular.z * m_dAxisLength) / 2) * 2 / m_dDiam; 
	// w2:
	traj.joint_names.push_back("wheel_front_right_base_link");
	point.velocities[1] = (twist.linear.x - (twist.angular.z * m_dAxisLength) / 2) * 2 / m_dDiam; 
	traj.points.push_back(point);
}

void DiffDrive2WKinematics::setAxisLength(double dLength)
{
	m_dAxisLength = dLength;
}

void DiffDrive2WKinematics::setWheelDiameter(double dDiam)
{
	m_dDiam = dDiam;
}
