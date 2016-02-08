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
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>
#include <hrov_control/HrovControlStdMsg.h>

using namespace std;


class Hrov_control
{

	public:
		Hrov_control();
		~Hrov_control();
		
		bool 		robotCollision;
		bool		userControlRequest;
		int			missionType;
		int			safetyAlarm;
		int			blackboxPhase[4];
		
		geometry_msgs::Pose blackboxPose;
		
	private:
		ros::NodeHandle		nh;

		ros::Subscriber		sub_userControlInfo;
		ros::Subscriber		sub_safetyInfo;
		
		ros::ServiceClient	runBlackboxGotoPoseSrv;

		geometry_msgs::Pose			robotCurrentPose;
		geometry_msgs::Pose			robotLastPose;
		geometry_msgs::PoseStamped  robotDesiredPosition;

		void missionMenu();
		void blackboxMenu();
		void blackboxPosition();
		void BlackboxGotoPose();
		void GoToSurface();
		void safetyMeasuresCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
		void userControlReqCallback(const std_msgs::Bool::ConstPtr& msg);

};
