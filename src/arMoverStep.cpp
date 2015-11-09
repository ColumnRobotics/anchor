/*
 * arMoverStep.cpp
 * 
 * Copyright 2015 Cole Gulino <cole_gulino@colegulino-ThinkPad-E450>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

// #include 
#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Twis.h>
#include <std_msgs/Empty.h>
#include <iostream>

const float32 LATERAL_SPEED = 0.1;
const float32 ROTATION_SPEED = 0.2;

class AnchorDrone{
public:
	AnchorDrone();
	~AnchorDrone();
	void stepLoop();
	
private:
	ros::NodeHandle nh_; // Standard ROS object to handle nodes
	ros::Publisher land_pub_; // Sandard ROS publisher object to publish std_msgs::Empty to /ardrone/land topic
	ros::Publisher reset_pub_; // Standard ROS publisher object to publish std_msgs::Empty to /ardrone/reset topic
	ros::Publisher takeoff_pub_; // Standard ROS publisher object to publish std_msgs::Empty to /ardrone/takeoff topic
	ros::Publisher flattrim_pub_; // Standard ROS publisher object to publish std_msgs::Empty to /ardrone/flattrim topic
	ros::Publisher cmd_pub_; // Standard ROS publisher object to publish geometry_msgs::Twist to /cmd_vel topic
	geometry_msgs::Twist cmd;
};

AnchorDrone::AnchorDrone(){
	// Initialize the publishers
	cmd_pub_ = nh_.advertise<geometry_msgs::Twist>( "/cmd_vel", 10 );
	land_pub_ = nh.advertise<std_msgs::Empty>( "/ardrone/land", 10 );
	reset_pub_ = nh_.advertise<std_msgs::Empty>( "/ardrone/reset", 10 );
	takeoff_pub_ = nh.advertise<std_msgs::Empty>( "/ardrone/takeoff", 10 );
	flattrim_pub_ = nh.advertise<std_msgs::Empty>( "/ardrone/flattrim", 10 );
}

AnchorDrone::~AnchorDrone(){ }

AnchorDrone::stepLoop(){
	// create a std_msgs::Empty object
	std_msgs::Empty empty;
	
	// Get current time
	ros::Time begin = ros::Time::now();
	
	// Publish to flattrim in order to reset gyros
	flattrim_pub_.publish( empty );
	
	// Take off
	takeoff_pub_.publish( empty );
	
	// Go forward for 2 seconds
	while( ros::Time::now() - begin < 2 ){ 
		// Send forward command velocity
		cmd.linear.x = LATERAL_SPEED;
		cmd.linear.y = 0;
		cmd.linear.z = 0;
		cmd.angular.x = 0;
		cmd.angular.y = 0;
		cmd.angular.z = 0;
		cmd_pub_.publish( cmd );
	}
}

int main(int argc, char **argv){
	ros::init( argc, argv, "anchorDrone" );
	ros::Rate loop_rate(40); // Set frequency to 40 Hz
	AnchorDrone anchorDrone;
	anchorDrone.stepLoop();
	return 0;
}

