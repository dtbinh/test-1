/**
 * \file velocity_node.cpp
 * \brief A node to compute desired twist to control a robot.
 * \author Alessandro Carfì and Rafał Kosk
 * \version 0.1
 * \date 23 June 2015
 * 
 *
 *
 * Subscribes to: <BR>
 *    ° Absolute topic "/key_typed" <BR>
 * 
 * Publishes to: <BR>
 *    ° Absolute topic "/vel_keyobard"
 *
 * This ROS node is used to set the desired twist konowing witch key is pressed.
 * Up arrow: increase linear velocity along x;
 * Down arrow: decrease linear velocity along x;
 * Left arrow: increase angular velocity along z;
 * Right arrow: decrease angular velocity along z;
 *
 */



//ROS
#include "ros/ros.h"

//ROS msgs
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>


const int up = 65;
const int down = 66;
const int left = 68;
const int right = 67;

const float Max_linear = 0.2;
const float Max_angular = 1;
float actual_linear = 0;
float actual_angular = 0;

	
void keyHitCallback(std_msgs::Int16 msg_key_hit){
	if(msg_key_hit.data == up){
		if(actual_linear < (Max_linear-Max_linear/10)){
			actual_linear += Max_linear/10;
		}
	}
	
	if(msg_key_hit.data == down){
		if(actual_linear > 0){
			actual_linear -= Max_linear/10;
		}
	}
	
	if(msg_key_hit.data == left){
		if(actual_angular < Max_angular){
			actual_angular += Max_angular/10;
		}
	}
	
	if(msg_key_hit.data == right){
		if(actual_angular > -Max_angular){
			actual_angular -= Max_angular/10;
		}
	}	
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "velocity_node");
	ROS_INFO("Connected !!!!!");
	ros::NodeHandle node("~");
	
	if(!ros::master::check()) return(0);
	ROS_INFO("velocity_node just started");
	
	/* ***************
	   *SUBSCRIPTIONS*
	   *************** */
	
	// 1. Sbscribe to the pressed key
	ros::Subscriber key_hit = node.subscribe<std_msgs::Int16> ("/key_typed"  , 1, keyHitCallback);
	
	/* ************
	   *PUBLISHING*
	   ************ */
	
	// 2. Publish the desired tiwst for the leader
	ros::Publisher pub_vel = node.advertise<geometry_msgs::Twist>("/vel_keyobard", 1);
	
	ros::Rate rate(100);
	while (ros::ok()){
		
		ros::spinOnce();
		geometry_msgs::Twist actual_twist;
		
		actual_twist.linear.x = actual_linear;
		actual_twist.linear.y = 0;
		actual_twist.linear.z = 0;
		actual_twist.angular.x = 0;
		actual_twist.angular.y = 0;
		actual_twist.angular.z = actual_angular;
		
		pub_vel.publish(actual_twist);
		
		rate.sleep();
	} 

}