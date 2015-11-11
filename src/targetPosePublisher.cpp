/*
 * targetPosePublisher.cpp
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
 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

int main(int argc, char **argv){
	ros::init( argc, argv, "targetPosePublisher");
	
	ros::NodeHandle n;
	ros::Publisher targetPosePub = n.advertise<geometry_msgs::Pose>("targetPose", 1);
	geometry_msgs::Pose targetPose;
	
	targetPose.position.x = 2;
	targetPose.position.x = 1;
	targetPosePub.publish( targetPose );
	
	ros::spin();
	return 0;
}

