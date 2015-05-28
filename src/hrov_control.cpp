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

//DEBUG Flags
#define DEBUG_FLAG	0

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Hrov_control");
  Hrov_control HrovControl;
  ros::spin();
}


Hrov_control::Hrov_control()
{
	robotLastPose.position.x = 0; robotLastPose.position.y = 0; robotLastPose.position.z = 0;
//	userControlRequest.data = false;
	missionType = 0;

	for (int i=0; i<4; i++)
		blackboxPhase[i] = 0;
	
	//Services initialization
	runBlackboxGotoPoseSrv = nh_.serviceClient<hrov_control::HrovControlStdMsg>("runBlackboxGotoPoseSrv");


	
	
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
	int i;
	geometry_msgs::Pose blackboxPose;

	cout << "HROV mission control menu" << endl;
	cout << "-------------------------" << endl;
	cout << "Select a mission type" << endl;
	cout << "1) Survey" << endl;
	cout << "2) Blackbox recovery" << endl;
	cout << "3) Panel intervention" << endl;
	cout << "9) Test the system" << endl;
	cout << "0) Exit" << endl;
	
	cin >> i;
	if ((i >= 0) and (i <= 9))
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
		case 9:
			cout << "Testing the system..." << endl;
			break;
		
	}

}


/************************************************************************/
/*						BLACKBOX RECOVERY								*/
/************************************************************************/


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
		cout << "blackboxPhase[0] == 0" << endl;
}


void Hrov_control::BlackboxGotoPose()
{
	hrov_control::HrovControlStdMsg startStopSrv;

	startStopSrv.request.boolValue = true;
	startStopSrv.request.robotTargetPosition.position.x = robotDesiredPosition.pose.position.x;
	startStopSrv.request.robotTargetPosition.position.y = robotDesiredPosition.pose.position.y;
	startStopSrv.request.robotTargetPosition.position.z = robotDesiredPosition.pose.position.z;

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


void Hrov_control::GoToSurface()
{
	robotDesiredPosition.pose.position.x = 0;
	robotDesiredPosition.pose.position.y = 0;
	robotDesiredPosition.pose.position.z = 0;
}
