/* 
 * Copyright (c) 2015 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Author:
 *     Juan Carlos García
 */ 

#include <iostream>
#include <stdlib.h>
#include <string.h>

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <hrov_control/HrovControlStdMsg.h>

using namespace std;


class Hrov_control
{

	public:
		Hrov_control();
		~Hrov_control();
		
		bool 		userControl;
		bool 		robotCollision;
		int			missionType;
		int			blackboxPhase[4];
		
		ros::Time	lastPress;

		geometry_msgs::Pose blackboxPose;
		
		
	private:
		ros::NodeHandle nh_;

		ros::Publisher		goto_pub_;
		ros::Subscriber		joystick_sub_;
		ros::Subscriber		odom_sub_;
		ros::ServiceClient	runBlackboxGotoPoseSrv;

		geometry_msgs::Pose	robotCurrentPose;
		geometry_msgs::Pose robotDesiredPose;

		void odomCallback(const geometry_msgs::Pose::ConstPtr& odomValue);
		void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);
		void missionMenu();
		void blackboxPosition();
		bool BlackboxGotoPose();


};
