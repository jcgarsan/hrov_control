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
#include "../include/hrov_control/hrov_control.h"


using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Hrov_control");
  Hrov_control HrovControl;
  ros::spin();
}


Hrov_control::Hrov_control()
{
	userControlRequest.data = false;
	missionType = 0;
	lastPress = ros::Time::now();

	for (int i=0; i<4; i++)
		blackboxPhase[i] = 0;
	
	//Publisher initialization
	goto_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("gotopose", 1);
	userControlRequest_pub_ = nh_.advertise<std_msgs::Bool>("userControlRequest", 1);

	//Subscriber initialization by device to be used
	joystick_sub_ = nh_.subscribe<sensor_msgs::Joy>("joystick_out", 1, &Hrov_control::joystickCallback, this); 
	odom_sub_ = nh_.subscribe<geometry_msgs::Pose>("g500/pose", 1, &Hrov_control::odomCallback, this);

	//Services initialization
	runBlackboxGotoPoseSrv = nh_.serviceClient<hrov_control::HrovControlStdMsg>("runBlackboxGotoPoseSrv");


	
	
	missionMenu();
}


Hrov_control::~Hrov_control()
{
	//Destructor
}


void Hrov_control::missionMenu()
{
	int i;
	geometry_msgs::Pose blackboxPose;

	cout << "HROV mission control menu" << endl;
	cout << "-------------------------" << endl;
	cout << "Select a mission type" << endl;
	cout << "1) Survey" << endl;
	cout << "2) Blackbox recovery" << endl;
	cout << "3) Panel intervention" << endl;
	cout << "0) Exit" << endl;
	
	cin >> i;
	if ((i >= 0) and (i <= 3))
		missionType = i;

	switch (missionType)
	{
		case 0:
			cout << "Program finished..." << endl;
			break;
		case 1:
			cout << "Survey specification menu..." << endl;
			break;
		case 2:
			cout << "Blackbox recovery mission..." << endl;
			blackboxPosition();
			break;
		case 3:
			cout << "Panel intervention..." << endl;
			break;
		
	}

}


void Hrov_control::blackboxPosition()
{
	cout << "Starting phase 1: Go to target position..." << endl;
	cout << "Enter blackbox x-location: ";
	cin >> blackboxPose.position.x;
	cout << "Enter blackbox y-location: ";
	cin >> blackboxPose.position.y;
	cout << "Enter blackbox z-location: ";
	cin >> blackboxPose.position.z;
	cout << endl;
	
	//Publish GoToPose info: where the robot should go?
	robotDesiredPose.header.stamp = ros::Time::now();
	robotDesiredPose.pose.position.x = blackboxPose.position.x - robotCurrentPose.position.x;
	robotDesiredPose.pose.position.y = blackboxPose.position.y - robotCurrentPose.position.y;
	robotDesiredPose.pose.position.z = blackboxPose.position.z - robotCurrentPose.position.z;
	goto_pub_.publish(robotDesiredPose);
	
	if (blackboxPhase[0] == 0)
		BlackboxGotoPose();
	else
		cout << "blackboxPhase[0] == 0" << endl;
}


void Hrov_control::BlackboxGotoPose()
{
	hrov_control::HrovControlStdMsg startStopSrv;

	startStopSrv.request.boolValue = true;

	if (runBlackboxGotoPoseSrv.call(startStopSrv))
	{
		ROS_INFO_STREAM("Finished mission: " << startStopSrv.response);
		if (startStopSrv.response.boolValue)
		{
			blackboxPhase[0] = 1;
			cout << "blackboxPhase[] = " ;
			for (int i=0; i<4; i++)
				cout << blackboxPhase[i] << ", ";
			cout << endl;
		}
		else
		{
			cout << "blackboxPhase[] = " ;
			for (int i=0; i<4; i++)
				cout << blackboxPhase[i] << ", ";
			cout << endl;
		}
	}
	else
	{
		ROS_INFO_STREAM("Failed to call service runBlackboxGotoPoseSrv");
	}
}



void Hrov_control::odomCallback(const geometry_msgs::Pose::ConstPtr& odomValue)
{
//	cout << "Position\n" << odomValue->position << "\nOrientation\n" << odomValue->orientation << endl;
	robotCurrentPose.position = odomValue->position;
	robotCurrentPose.orientation = odomValue->orientation;
}


void Hrov_control::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
	ros::Time currentPress = ros::Time::now();
	ros::Duration difTime = currentPress - lastPress;
	if ((difTime.toSec() > 0.5) and (joystick->buttons[0] == 1))
	{
		userControlRequest.data = !userControlRequest.data;
		lastPress = currentPress;
		cout << "userControlRequest button pressed. userControlRequest = " << userControlRequest << endl;
		userControlRequest_pub_.publish(userControlRequest);
	}
}
