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

#define pressureThreshold	0.5
#define rangeThreshold 		1.0
#define num_sensors			2		// 0 = is there an alarm?, 1 = surface, 2 = seafloor

//DEBUG Flags
#define DEBUG_FLAG			0
#define DEBUG_FLAG_SAFETY	0

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Hrov_control");
	Hrov_control HrovControl;
	ros::spin();
  
}


Hrov_control::Hrov_control()
{
	userControlRequestAlarm  = false;
	userControlRequestButton = false;
	robotLastPose.position.x = 0; robotLastPose.position.y = 0; robotLastPose.position.z = 0;
	missionType = 0;

	for (int i=0; i<=num_sensors+1; i++)
		safetyMeasureAlarm.data.push_back(0);


	for (int i=0; i<4; i++)
		blackboxPhase[i] = 0;

	//Subscribers initialization
	sub_userControlInfo = nh.subscribe<std_msgs::Bool>("userControlRequest", 1, &Hrov_control::userControlReqCallback, this);
	sub_sensorPressure = nh.subscribe<underwater_sensor_msgs::Pressure>("g500/pressure", 1, &Hrov_control::sensorPressureCallback, this);
	sub_sensorRange = nh.subscribe<sensor_msgs::Range>("uwsim/g500/range", 1, &Hrov_control::sensorRangeCallback, this);
	
	//Publishers initialization
    pub_safety = nh.advertise<std_msgs::Int8MultiArray>("safetyMeasures", 1);

	missionMenu();
}


Hrov_control::~Hrov_control()
{
	//Destructor
}


/************************************************************************/
/*						COMMON FUNCTIONS								*/
/************************************************************************/

void Hrov_control::missionMenu()
{
	cout << "\n\rHROV mission control menu" << endl;
	cout << "-------------------------" << endl;
	cout << "Select a mission type" << endl;
	cout << "1) Survey" << endl;
	cout << "2) Blackbox recovery" << endl;
	cout << "3) Panel intervention" << endl;
	cout << "8) Go to surface" << endl;
	cout << "9) Test the system" << endl;
	cout << "0) Exit" << endl;
	
	cout << "Mission type: ";
	cin >> missionType;
	if ((missionType >= 0) and (missionType <= 9))
	{
		switch (missionType)
		{
			case 0:
				cout << "Program finished..." << endl;
				exit(0);
				break;
			case 1:
				cout << "Survey specification menu..." << endl;
				break;
			case 2:
				cout << "Blackbox recovery mission..." << endl;
				blackboxMenu();
				break;
			case 3:
				cout << "Panel intervention..." << endl;
				break;
			case 8:
				cout << "Go to surface..." << endl;
				GoToSurface();
				break;
			case 9:
				cout << "Testing the system..." << endl;
				break;
		}
	}
	else
	{
		cout << "missionType should be between 0...9" << endl;
		missionMenu();
	}
}


/************************************************************************/
/*						BLACKBOX RECOVERY								*/
/************************************************************************/
void Hrov_control::blackboxMenu()
{
	cout << "\n\rBlackbox recovery menu" << endl;
	cout << "-------------------------" << endl;
	cout << "Select a mission type" << endl;
	cout << "1) Set target position" << endl;
	cout << "2) Init blackbox detection" << endl;
	cout << "0) Exit" << endl;

	cout << "Mission type: ";
	cin >> missionType;
	if ((missionType >= 0) and (missionType <= 2))
	{
		switch (missionType)
		{
			case 0:
				cout << "Program finished..." << endl;
				exit(0);
				break;
			case 1:
				blackboxPhase[0] = 0;
				blackboxPosition();
				break;
			case 2:
				cout << "Blackbox detection algorithm..." << endl;
				break;
		}		
	}
	else
	{
		cout << "missionType should be between 0...2" << endl;
		blackboxMenu();
	}
}


void Hrov_control::blackboxPosition()
{
	cout << "Starting phase 1: Go to target position..." << endl;
	cout << "Enter blackbox x-location: ";
	cin >> robotDesiredPosition.pose.position.x;
	cout << "Enter blackbox y-location: ";
	cin >> robotDesiredPosition.pose.position.y;
	cout << "Enter blackbox z-location: ";
	cin >> robotDesiredPosition.pose.position.z;
	cout << endl;
	
	if (blackboxPhase[0] == 0)
		BlackboxGotoPose();
	else
	{
		cout << "BlackboxGotoPose() finished. blackboxPhase[0] == 0" << endl;
	}
}


void Hrov_control::BlackboxGotoPose()
{
	actionlib::SimpleActionClient<thruster_control::goToPoseAction> ac("GoToPoseAction", true);
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	
	ROS_INFO("Waiting for action server to start");
	ac.waitForServer();	
	goal.boolValue = true;
	goal.robotTargetPosition.position.x = robotDesiredPosition.pose.position.x;
	goal.robotTargetPosition.position.y = robotDesiredPosition.pose.position.y;
	goal.robotTargetPosition.position.z = robotDesiredPosition.pose.position.z;
	ac.sendGoal(goal);
}


void Hrov_control::GoToSurface()
{
	actionlib::SimpleActionClient<thruster_control::goToPoseAction> ac("GoToPoseAction", true);
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	
	ROS_INFO("Waiting for action server to start");
	ac.waitForServer();	
	goal.boolValue = true;
	goal.robotTargetPosition.position.x = 0;
	goal.robotTargetPosition.position.y = 0;
	goal.robotTargetPosition.position.z = 2;
	ac.sendGoal(goal);
}


/************************************************************************/
/*							CALLBACKS									*/
/************************************************************************/
//Pressure sensor = safetyAlarm[1]
void Hrov_control::sensorPressureCallback(const underwater_sensor_msgs::Pressure::ConstPtr& pressureValue)
{
	if (abs(pressureValue->pressure) < pressureThreshold)
	{
		safetyMeasureAlarm.data[0]	= 1;
		safetyMeasureAlarm.data[1]	= 1;
		userControlRequestAlarm		= true;
	}
	else
		safetyMeasureAlarm.data[1] = 0;

	if ((safetyMeasureAlarm.data[1] == 0) and (safetyMeasureAlarm.data[2] == 0))
	{
		safetyMeasureAlarm.data[0]	= 0;
		userControlRequestAlarm		= false;
	}

	pub_safety.publish(safetyMeasureAlarm);
		
	if (DEBUG_FLAG_SAFETY)
		cout << "sensorPressureAlarm: " << abs(pressureValue->pressure) << " :: " << (int) safetyMeasureAlarm.data[1] << endl;
}


//Range sensor = safetyAlarm[2]
void Hrov_control::sensorRangeCallback(const sensor_msgs::Range::ConstPtr& rangeValue)
{
	if (rangeValue->range < rangeThreshold)
	{
		safetyMeasureAlarm.data[0]	= 1;
		safetyMeasureAlarm.data[2]	= 1;
		userControlRequestAlarm		= true;
	}
	else
		safetyMeasureAlarm.data[2] = 0;
	
	if ((safetyMeasureAlarm.data[1] == 0) and (safetyMeasureAlarm.data[2] == 0))
	{
		safetyMeasureAlarm.data[0]	= 0;
		userControlRequestAlarm		= false;
	}
	
	pub_safety.publish(safetyMeasureAlarm);

	if (DEBUG_FLAG_SAFETY)
		cout << "sensorRangeAlarm: " << rangeValue->range << " :: " << (int) safetyMeasureAlarm.data[2] << endl;
}


//User control request = safetyAlarm[3]
void Hrov_control::userControlReqCallback(const std_msgs::Bool::ConstPtr& msg)
{
	//Has the user requested the control using the button?
	userControlRequestButton = msg->data;

	//We create the alarm if the user pushed the button or there is an alarm
	if ((userControlRequestButton) or (userControlRequestAlarm))
		safetyMeasureAlarm.data[num_sensors+1] = 1;
	else
		safetyMeasureAlarm.data[num_sensors+1] = 0;

	pub_safety.publish(safetyMeasureAlarm);

	if (DEBUG_FLAG_SAFETY)
		cout << "userControlRequestCallback: " << (int) safetyMeasureAlarm.data[num_sensors+1] << endl;
}


