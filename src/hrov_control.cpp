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
#define num_sensors			2		// #0 = is there an alarm?, #1 = surface, #2 = seafloor

//DEBUG Flags
#define DEBUG_FLAG_SAFETY	0
#define DEBUG_FLAG_USER		0

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
	armControlRequestAlarm	 = false;
	armControlRequestButton  = false;
	goToPoseAcResult		 = false;
	stKeeping				 = false;
	robotControl			 = true;
	robotLastPose.position.x = 0; robotLastPose.position.y = 0; robotLastPose.position.z = 0;
	missionType = 0;

	for (int i=0; i<=num_sensors; i++)
		safetyMeasureAlarm.data.push_back(0);

	for (int i=0; i<2; i++)
		userControlAlarm.data.push_back(0);

	for (int i=0; i<6; i++)
	{
		objectRecoveryPhase[i]	= 0;
		dredgingPhase[i]		= 0;
	}
	
	//Subscribers initialization
	sub_userControlRequest = nh.subscribe<std_msgs::Bool>("userControlRequest", 1, &Hrov_control::userControlReqCallback, this);
	sub_armControlRequest = nh.subscribe<std_msgs::Bool>("armControlRequest", 1, &Hrov_control::armControlReqCallback, this);
	sub_sensorPressure = nh.subscribe<underwater_sensor_msgs::Pressure>("g500/pressure", 1, &Hrov_control::sensorPressureCallback, this);
	sub_sensorRange = nh.subscribe<sensor_msgs::Range>("uwsim/g500/range", 1, &Hrov_control::sensorRangeCallback, this);
	sub_goToPoseActionResult = nh.subscribe<thruster_control::goToPoseActionResult>("GoToPoseAction/result", 1, &Hrov_control::goToPoseAcResultCallback, this);
	
	//Publishers initialization
    pub_safety = nh.advertise<std_msgs::Int8MultiArray>("safetyMeasuresAlarm", 1);
    pub_userControl = nh.advertise<std_msgs::Int8MultiArray>("userControlAlarm", 1);

	//ACtion client initialization
	ac = new actionlib::SimpleActionClient<thruster_control::goToPoseAction> ("GoToPoseAction", true);

	missionMenu();
}


Hrov_control::~Hrov_control()
{
	//Destructor
	delete[] ac;
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
	cout << "2) Object recovery" << endl;
	cout << "3) Panel intervention" << endl;
	cout << "4) Dredging intervention" << endl;
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
				cout << "Object recovery mission..." << endl;
				objectRecoveryMenu();
				break;
			case 3:
				cout << "Panel intervention..." << endl;
				break;
			case 4:
				cout << "Dredging intervention..." << endl;
				dredgingMenu();
				break;
			case 8:
				cout << "Go to surface..." << endl;
				goToSurface();
				break;
			case 9:
				systemTest();
				break;
		}
	}
	else
	{
		cout << "missionType should be between 0...9" << endl;
		missionMenu();
	}
}


void Hrov_control::systemTest()
{
	bool enableTesting = true;
	userControlRequestAlarm	= true;

	cout << "Testing the system. Press userControlButton to test the system." << endl;
	sleep(5);
	userControlAlarm.data[0] = 1;
	pub_safety.publish(userControlAlarm);

	cout << "Press userControlButton to return main menu." << endl;
	while (enableTesting)
	{
		ros::spinOnce();
		enableTesting = userControlRequestButton;
	}

	missionMenu();

}

/************************************************************************/
/*						 OBJECT RECOVERY								*/
/************************************************************************/
void Hrov_control::objectRecoveryMenu()
{
	cout << "\n\rObject recovery menu" << endl;
	cout << "-------------------------" << endl;
	cout << "Select an option" << endl;
	cout << "1) Set target position" << endl;
	cout << "2) Init target detection" << endl;
	cout << "3) Vehicle station keeping" << endl;
	cout << "0) Exit" << endl;

	cout << "Mission type: ";
	cin >> missionType;
	if ((missionType >= 0) and (missionType <= 3))
	{
		switch (missionType)
		{
			case 0:
				cout << "Program finished..." << endl;
				exit(0);
				break;
			case 1:
				objectPosition();
				break;
			case 2:
				cout << "Target detection algorithm..." << endl;
				objectRecoveryPhase[1] = 1;
				objectRecoveryMenu();
				break;
			case 3:
				cout << "Keeping current robot position..." << endl;
				stationKeeping();
				break;
		}		
	}
	else
	{
		cout << "missionType should be between 0...3" << endl;
		objectRecoveryMenu();
	}
}


/************************************************************************/
/*						 	DREDGING									*/
/************************************************************************/
void Hrov_control::dredgingMenu()
{
	cout << "\n\rDredging intervention menu" << endl;
	cout << "-------------------------" << endl;
	cout << "Select an option" << endl;
	cout << "1) Set target position" << endl;
	cout << "2) Init target detection" << endl;
	cout << "3) Vehicle station keeping" << endl;
	cout << "4) Manual dredging" << endl;
	cout << "0) Exit" << endl;

	cout << "Mission type: ";
	cin >> missionType;
	if ((missionType >= 0) and (missionType <= 3))
	{
		switch (missionType)
		{
			case 0:
				cout << "Program finished..." << endl;
				exit(0);
				break;
			case 1:
				objectPosition();
				break;
			case 2:
				cout << "Target detection algorithm..." << endl;
				dredgingPhase[1] = 1;
				dredgingMenu();
				break;
			case 3:
				cout << "Keeping current robot position..." << endl;
				stationKeeping();
				break;
			case 4:
				cout << "Manual dredging..." << endl;
				break;
		}		
	}
	else
	{
		cout << "missionType should be between 0...4" << endl;
		objectRecoveryMenu();
	}
}


void Hrov_control::objectPosition()
{
	cout << "Starting phase 1: Go to target position..." << endl;
	cout << "Enter target x-location: ";
	cin >> robotDesiredPosition.pose.position.x;
	cout << "Enter target y-location: ";
	cin >> robotDesiredPosition.pose.position.y;
	cout << "Enter target z-location: ";
	cin >> robotDesiredPosition.pose.position.z;
	cout << endl;
	
	if ((objectRecoveryPhase[0] == 1) or (dredgingPhase[0] == 1))
		cout << "The object position was set previously and it will be reset" << endl;

	objectGotoPose();
}


void Hrov_control::objectGotoPose()
{
//	actionlib::SimpleActionClient<thruster_control::goToPoseAction> ac("GoToPoseAction", true);
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	objectRecoveryPhase[0] = 0;
	dredgingPhase[0] = 0;
	
	ROS_INFO("Waiting for action server to start");
	ac->waitForServer();	
	goal.startAction = true;
	goal.stationKeeping = false;
	goal.robotTargetPosition.position.x = robotDesiredPosition.pose.position.x;
	goal.robotTargetPosition.position.y = robotDesiredPosition.pose.position.y;
	goal.robotTargetPosition.position.z = robotDesiredPosition.pose.position.z;
	ac->sendGoal(goal);
	ROS_INFO("Action sent to server");
}


void Hrov_control::stationKeeping()
{
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	
	ROS_INFO("Waiting for action server to start");
	ac->waitForServer();	
	goal.startAction = true;
	goal.stationKeeping = true;
	goal.robotTargetPosition.position.x = robotDesiredPosition.pose.position.x;
	goal.robotTargetPosition.position.y = robotDesiredPosition.pose.position.y;
	goal.robotTargetPosition.position.z = robotDesiredPosition.pose.position.z;
	ac->sendGoal(goal);
	ROS_INFO("Action sent to server");
	
	objectRecoveryPhase[2]	= 1;
	dredgingPhase[2]		= 1;
	missionMenu();
}


void Hrov_control::goToSurface()
{
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	
	ROS_INFO("Waiting for action server to start");
	ac->waitForServer();	
	goal.startAction = true;
	goal.stationKeeping = false;
	goal.robotTargetPosition.position.x = 0;
	goal.robotTargetPosition.position.y = 0;
	goal.robotTargetPosition.position.z = 2;
	ac->sendGoal(goal);
	ROS_INFO("Action sent to server");
	
	objectRecoveryPhase[4]	= 1;
	dredgingPhase[4]		= 1;
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


//User control request = userControl[0]
void Hrov_control::userControlReqCallback(const std_msgs::Bool::ConstPtr& msg)
{
	//Has the user requested the vehicle control using the button?
	userControlRequestButton = msg->data;

	//We create the alarm if the user pushed the button or there is an alarm
	if ((userControlRequestButton) or (userControlRequestAlarm))
		userControlAlarm.data[0] = 1;
	else
		userControlAlarm.data[0] = 0;

	pub_userControl.publish(userControlAlarm);

	if (DEBUG_FLAG_USER)
		cout << "userControlRequestCallback: " << (int) userControlAlarm.data[0] << endl;
}


//Arm control request = userControl[1]
void Hrov_control::armControlReqCallback(const std_msgs::Bool::ConstPtr& msg)
{
	//Has the user requested the arm control using the button?
	armControlRequestButton = msg->data;

	//We create the alarm if the user pushed the button or there is an alarm
	if ((armControlRequestButton) or (armControlRequestAlarm))
		userControlAlarm.data[1] = 1;
	else
		userControlAlarm.data[1] = 0;

	pub_userControl.publish(userControlAlarm);

	if (DEBUG_FLAG_USER)
		cout << "armControlRequestCallback: " << (int) userControlAlarm.data[1] << endl;
}

//Check if the GoToPoseAction is finished
void Hrov_control::goToPoseAcResultCallback(const thruster_control::goToPoseActionResult::ConstPtr& msg)
{
	int backMenu;
	goToPoseAcResult = msg->result.succeed;
	
	if (goToPoseAcResult)
	{
		ROS_INFO("Action finished successfully.");
		objectRecoveryPhase[0] = 1;
		dredgingPhase[0] = 1;
		//missionMenu();
	}
	else
	{
		ROS_INFO("Action did not finish successfully or has been aborted by the user.");
		userControlRequestAlarm = true;
		//ROS_INFO("The user has the robot control. Enter 0 to return the main menu...");
		//cin >> backMenu;
		//if(backMenu == 0) 
			//missionMenu();
	}
	missionMenu();
	
	if (DEBUG_FLAG_SAFETY)
		cout << "goToPoseAcResult: " << (int) msg->result.succeed << endl;
	
}
