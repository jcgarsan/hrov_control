/* 
 * Copyright (c) 2014 University of Jaume-I.
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
	userControl = false;
	
	//Publisher initialization
	goto_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/gotopose", 1);
	
	//Subscriber initialization by device to be used
	joystick_sub_ = nh_.subscribe<sensor_msgs::Joy>("joystick_out", 1, &Hrov_control::joystickCallback, this); 
	odom_sub_ = nh_.subscribe<geometry_msgs::Pose>("g500/pose", 1, &Hrov_control::odomCallback, this);

	lastPress = ros::Time::now();

}

Hrov_control::~Hrov_control()
{
	//Destructor
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
		userControl = !userControl;
		lastPress = currentPress;
		cout << "userControl button pressed. userControl = " << userControl << endl;
	}
}
