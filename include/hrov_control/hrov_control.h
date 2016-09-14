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

#include <sstream>

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <underwater_sensor_msgs/Pressure.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <thruster_control/goToPoseAction.h>

using namespace std;


class Hrov_control
{

	public:
		Hrov_control();
		~Hrov_control();
		
		bool	userControlRequestAlarm;
		bool	userControlRequestButton;
		bool	armControlRequestAlarm;
		bool	armControlRequestButton;
		bool	goToPoseAcResult;
		bool	stKeeping;
		bool	robotControl;
		int		missionType;
		int 	function;
		int		objectRecoveryPhase[6];
		int		dredgingPhase[6];
		
		geometry_msgs::Pose 		objectPose;
		
		std_msgs::Int8MultiArray	safetyMeasureAlarm;
		std_msgs::Int8MultiArray	userControlAlarm;
		std_msgs::Int8MultiArray	userMenuData;
		std_msgs::Bool				armControlRequest;
		std_msgs::Int8				missionControlAlarm;
		
		actionlib::SimpleActionClient<thruster_control::goToPoseAction> *ac;

	private:
		ros::NodeHandle		nh;

		ros::Subscriber		sub_userControlRequest;
		ros::Subscriber		sub_armControlRequest;
		ros::Subscriber 	sub_sensorPressure;
		ros::Subscriber 	sub_sensorRange;
		ros::Subscriber		sub_goToPoseActionResult;
		ros::Subscriber		sub_robotRealPose;
		ros::Subscriber		sub_userMenu;

		ros::Publisher  	pub_safety;
		ros::Publisher  	pub_userControl;
		ros::Publisher  	pub_missionControl;
		ros::Publisher  	pub_hudFeedback;
		
		geometry_msgs::Pose			robotCurrentPose;
		geometry_msgs::Pose			robotLastPose;
		geometry_msgs::Pose 		robotRealPose;
		geometry_msgs::PoseStamped  robotDesiredPosition;

		void missionMenu();
		void systemTest();
		void manualArmControl();
		void objectRecoveryMenu();
		void dredgingMenu();
		void objectPosition();
		void objectGotoPose();
		void goToSurface();
		void stationKeeping();
		void disableStationKeeping();
		void userMenuControl();
		void userControlReqCallback(const std_msgs::Bool::ConstPtr& msg);
		void armControlReqCallback(const std_msgs::Bool::ConstPtr& msg);
		void sensorPressureCallback(const underwater_sensor_msgs::Pressure::ConstPtr& pressureValue);
		void sensorRangeCallback(const sensor_msgs::Range::ConstPtr& rangeValue);
		void goToPoseAcResultCallback(const thruster_control::goToPoseActionResult::ConstPtr& msg);
		void getRealRobotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
		void getUserMenuData(const std_msgs::Int8MultiArray::ConstPtr& msg);
		void userFeedback(const char *userMessage);

};
