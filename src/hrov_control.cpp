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

#define userMenuAcitvated	1

//DEBUG Flags
#define DEBUG_FLAG_SAFETY	0
#define DEBUG_FLAG_USER		0
#define DEBUG_FLAG_MENU		0

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
	missionControlAlarm.data = -1;
	robotLastPose.position.x = 0; robotLastPose.position.y = 0; robotLastPose.position.z = 0;
	missionType = 0;

	for (int i=0; i<=num_sensors; i++)
		safetyMeasureAlarm.data.push_back(0);

	for (int i=0; i<2; i++)
		userControlAlarm.data.push_back(0);

	for (int i=0; i<5; i++)
		userMenuData.data.push_back(0);
	
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
	sub_robotRealPose = nh.subscribe<geometry_msgs::Pose>("g500/pose", 1, &Hrov_control::getRealRobotPoseCallback, this);
	sub_userMenu = nh.subscribe<std_msgs::Int8MultiArray>("userMenuData", 1, &Hrov_control::getUserMenuData, this);
	
	//Publishers initialization
    pub_safety = nh.advertise<std_msgs::Int8MultiArray>("safetyMeasuresAlarm", 1);
    pub_userControl = nh.advertise<std_msgs::Int8MultiArray>("userControlAlarm", 1);
    pub_missionControl = nh.advertise<std_msgs::Int8>("missionControlAlarm", 1);
    pub_hudFeedback = nh.advertise<std_msgs::String>("HUDfeedback", 1000);

	//ACtion client initialization
	ac = new actionlib::SimpleActionClient<thruster_control::goToPoseAction> ("GoToPoseAction", true);

	if (userMenuAcitvated)
		userMenuControl();
	else
		missionMenu();
}


Hrov_control::~Hrov_control()
{
	//Destructor
	delete[] ac;
}



/************************************************************************/
/*						USER MENU FUNCTIONS								*/
/************************************************************************/



void Hrov_control::userMenuControl()
{
	cout << "Using data from the user menu" << endl;
}


/************************************************************************/
/*						COMMON FUNCTIONS								*/
/************************************************************************/
void Hrov_control::missionMenu()
{
	cout << "\n\rHROV mission control menu" << endl;
	cout << "-------------------------" << endl;
	cout << "Select a mission type" << endl;
	cout << "0) Survey" << endl;
	cout << "1) Object recovery" << endl;
	cout << "2) Panel intervention" << endl;
	cout << "3) Dredging intervention" << endl;
	cout << "4) Go to surface" << endl;
	cout << "5) Test the system" << endl;
	cout << "6) Exit" << endl;
	
	cout << "Mission type: ";
	cin >> missionType;
	if ((missionType >= 0) and (missionType <= 9))
	{
		missionControlAlarm.data = -1;
		pub_missionControl.publish(missionControlAlarm);

		switch (missionType)
		{
			case 0:
				cout << "Survey specification menu..." << endl;
				break;
			case 1:
				cout << "Object recovery mission..." << endl;
				objectRecoveryMenu();
				break;
			case 2:
				cout << "Panel intervention..." << endl;
				break;
			case 3:
				cout << "Dredging intervention..." << endl;
				dredgingMenu();
				break;
			case 4:
				cout << "Go to surface..." << endl;
				goToSurface();
				break;
			case 5:
				systemTest();
				break;
			case 6:
				cout << "Program finished..." << endl;
				system("pkill roslaunch");
				exit(0);
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
	bool enableTesting		= true;
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

	if (userMenuAcitvated)
		userMenuControl();
	else
		missionMenu();
}


void Hrov_control::manualArmControl()
{
	bool enableTesting 		= true;
	armControlRequestAlarm  = true;

	cout << "Manual arm control. Press armControlButton to enable the arm control." << endl;
	sleep(5);
	userControlAlarm.data[1] = 1;
	pub_safety.publish(userControlAlarm);

	cout << "Press armControlButton to return main menu." << endl;
	while (enableTesting)
	{
		ros::spinOnce();
		enableTesting = armControlRequestButton;
	}

	armControlRequestAlarm = false;

	if (userMenuAcitvated)
		userMenuControl();
	else
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
	//cout << "0) Exit" << endl;

	cout << "Mission type: ";
	cin >> missionType;
	if ((missionType >= 0) and (missionType <= 3))
	{
		switch (missionType)
		{
			case 0:
				cout << "Program finished..." << endl;
				system("pkill roslaunch");
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
		dredgingMenu();
	}
}


void Hrov_control::objectPosition()
{
	if (userMenuAcitvated)
	{
		robotDesiredPosition.pose.position.x = 13;
		robotDesiredPosition.pose.position.y = 14;
		robotDesiredPosition.pose.position.z = 10;
		//cout << "Target position defined: {10, 5, 5}" << endl;
		cout << "Target position defined: {13, 14, 10}" << endl;
	}
	else
	{
		cout << "Starting phase 1: Go to target position..." << endl;
		cout << "Enter target x-location: ";
		cin >> robotDesiredPosition.pose.position.x;
		cout << "Enter target y-location: ";
		cin >> robotDesiredPosition.pose.position.y;
		cout << "Enter target z-location: ";
		cin >> robotDesiredPosition.pose.position.z;
		cout << endl;
	}

	if ((objectRecoveryPhase[0] == 1) or (dredgingPhase[0] == 1))
		cout << "The object position was set previously and it will be reset" << endl;

	objectGotoPose();
}


void Hrov_control::objectGotoPose()
{
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	
	dredgingPhase[0] 		 = 0;
	objectRecoveryPhase[0] 	 = 0;
	missionControlAlarm.data = -1;
	pub_missionControl.publish(missionControlAlarm);
	
	ROS_INFO("Waiting for action server to start");
	ac->waitForServer();	
	goal.startAction	= true;
	goal.stationKeeping = false;
	goal.robotTargetPosition.position = robotDesiredPosition.pose.position;
	ac->sendGoal(goal);
	ROS_INFO("Action sent to server");
}


void Hrov_control::stationKeeping()
{
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	
	ROS_INFO("Waiting for action server to start");
	ac->waitForServer();	
	goal.startAction	= true;
	goal.stationKeeping	= true;
	//goal.robotTargetPosition.position = robotDesiredPosition.pose.position;
	goal.robotTargetPosition.position = robotRealPose.position;
	ac->sendGoal(goal);
	ROS_INFO("Action sent to server");
	
	objectRecoveryPhase[2]	= 1;
	dredgingPhase[2]		= 1;
	stKeeping 				= true;

	if (userMenuAcitvated)
		userMenuControl();
	else
		missionMenu();	
}


void Hrov_control::disableStationKeeping()
{
	robotDesiredPosition.pose.position = robotRealPose.position;
	stKeeping = false;
	objectGotoPose();
}

void Hrov_control::goToSurface()
{
	thruster_control::goToPoseGoal goal;
	ros::spinOnce();
	
	missionControlAlarm.data = -1;
	pub_missionControl.publish(missionControlAlarm);
	
	ROS_INFO("Waiting for action server to start");
	ac->waitForServer();	
	goal.startAction 	= true;
	goal.stationKeeping = false;
	goal.robotTargetPosition.position.x = 0;
	goal.robotTargetPosition.position.y = 0;
	goal.robotTargetPosition.position.z = 2;
	ac->sendGoal(goal);
	ROS_INFO("Action sent to server");
	
	objectRecoveryPhase[4]	= 1;
	dredgingPhase[4]		= 1;

	if (userMenuAcitvated)
		userMenuControl();
	else
		missionMenu();	
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
	goToPoseAcResult = msg->result.succeed;
	missionControlAlarm.data = msg->status.status;


	pub_missionControl.publish(missionControlAlarm);
	
	if (goToPoseAcResult)
	{
		ROS_INFO("Action finished successfully.");
		objectRecoveryPhase[0] 	= 1;
		dredgingPhase[0] 		= 1;
	}
	else
	{
		ROS_INFO("Action did not finish successfully or has been aborted by the user.");
		userControlRequestAlarm = true;
	}

	if (userMenuAcitvated)
		userMenuControl();
	else
		missionMenu();	
	
	if (DEBUG_FLAG_SAFETY)
	{
		cout << "goToPoseAcResultSucceed: " << (int) msg->result.succeed << endl;
		cout << "goToPoseAcStatusStatus: " << (int) msg->status.status << endl;
	}
}


void Hrov_control::getRealRobotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	robotRealPose.position 	  = msg->position;
	robotRealPose.orientation = msg->orientation;

	//cout << "Odometry: \n" << robotRealPose << endl;
}


void Hrov_control::getUserMenuData(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
	//cout << "getUserMenuData" << endl;
	userMenuData.data = msg->data;
	function = userMenuData.data[3] * 10 + userMenuData.data[4];

	std::stringstream ss;

	if (userMenuData.data[2] == 1)
	{
		switch(function)
		{
			case 4:
				//cout << "Go to surface..." << endl;
				userFeedback("Go to surface...");
				goToSurface();
				break;
			case 5:
				//cout << "Testing the system..." << endl;
				userFeedback("Testing the system... ");
				systemTest();
				break;
			case 6:
				//cout << "Program finished..." << endl;
				userFeedback("Program finished...");
				system("pkill navigatorPIcont");
				system("pkill thrusterAllocat");
				system("pkill uial");
				system("pkill hrov_arm_contro");
				system("pkill roslaunch");
				exit(0);
				break;
			case 40:
				//cout << "Sending the robot to the target position..." << endl;
				userFeedback("Sending the robot to the target position...");
				objectPosition();
				break;
			case 42:
				if (!stKeeping)
				{
					//cout << "Enabling the stationKeeping algorithm..." << endl;
					userFeedback("Enabling the stationKeeping algorithm...");
					stationKeeping();
				}
				else
				{
					//cout << "Disabling the stationKeeping algorithm..." << endl;
					userFeedback("Disabling the stationKeeping algorithm...");
					disableStationKeeping();
				}
				break;
			case 43:
				//cout << "Enabling manual dredging..." << endl;
				userFeedback("Enabling manual dredging...");
				manualArmControl();
				break;
		}
	}

	if (DEBUG_FLAG_MENU)
		cout << "userMenuData.data[2]: " << (int) userMenuData.data[2] << ". Function to execute: " << function << endl;
}


void Hrov_control::userFeedback(const char *userMessage)
{
	std_msgs::String msg;

	msg.data = userMessage;
	pub_hudFeedback.publish(msg);

	ROS_INFO("%s", msg.data.c_str());

	ros::spinOnce();
}