/**
 * \file leade_control_node.cpp
 * \brief A node to compute desired wheels velocity for a (2,0) leader robot
 * \author Alessandro Carfì and Rafał Kosk
 * \version 0.1
 * \date 23 June 2015
 * 
 *
 * \param[in] "L": L, no default value.
 * \param[in] "radius": r, no default value.
 * \param[in] "LeftHandler": LeftHandler, no default value.
 * \param[in] "RightHandler": RightHandler, no default value.
 *
 * Subscribes to: <BR>
 *    ° Absolute topic "/leader_velocity" <BR>
 * 
 * Publishes to: <BR>
 *    ° Absolute topic "/speed_leader"
 *
 * This ROS node is used to compute desired wheels velocity for (2,0) robots controlled by an user
 * 
 */




//ROS
#include <ros/ros.h>

//ROS msg
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8MultiArray.h>

//V-rep
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosGetObjectHandle.h"

// Global variables (modified by topic subscribers):
geometry_msgs::Twist velocity;

// Topic subscriber callbacks:

void velocityCallback(const geometry_msgs::Twist _velocity) {
	velocity = _velocity;
}

// Main code:
int main(int argc,char* argv[])
{
	ros::init(argc,argv,"leader_control_node");
	ros::NodeHandle node("~");
	
	if(!ros::master::check()) return(0);
	ROS_INFO("leader_control_node just started");
	
	// The wheels handler and proximity sensor handler
	std::string LeftHandler,RightHandler;
	int handleL, handleR;
	
	//Phisical parameters
	double r ,L;

	//Parameters
	node.getParam("radius",r);
	node.getParam("L",L);

	node.getParam("LeftHandler",LeftHandler);
	node.getParam("RightHandler",RightHandler);
	
	/* ***************
	   *SUBSCRIPTIONS*
	   *************** */
	
	// 1. Subscribe to the twist
	ros::Subscriber subDir=node.subscribe("/leader_velocity",1,velocityCallback);

	/* ************
	   *PUBLISHING*
	   ************ */
	
	// 1. Speed of the wheels of the leader
	ros::Publisher speedPub=node.advertise<vrep_common::JointSetStateData>("/speed_leader",1);

	/* ***************
	   *V-REP_HANDLER*
	   *************** */
	
	//These are two service calls that returns the handlers of the wheels
	ros::ServiceClient client_getObjectHandle=node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
	vrep_common::simRosGetObjectHandle srv_getObjectHandle;
	srv_getObjectHandle.request.objectName=LeftHandler;
	if ( client_getObjectHandle.call(srv_getObjectHandle))
	{
	   handleL=srv_getObjectHandle.response.handle;
	}

	srv_getObjectHandle.request.objectName= RightHandler;
	if ( client_getObjectHandle.call(srv_getObjectHandle))
	{
	   handleR=srv_getObjectHandle.response.handle;
	}

	ros::Rate rate(100);
	while (ros::ok()){
		
		ros::spinOnce();
		
		//Convertion from linear and angular velocity to speed of the wheels
		//using the actuation matrix of the (2,0)
		double leftMotorSpeed = 1/r*velocity.linear.x - L/r*velocity.angular.z;
		double righMotorSpeed = 1/r*velocity.linear.x + L/r*velocity.angular.z ;

		vrep_common::JointSetStateData speed;
		
		speed.handles.data.push_back(handleL);
		speed.handles.data.push_back(handleR);
		speed.setModes.data.push_back(2); // 2 state for "speed mode"
		speed.setModes.data.push_back(2);
		speed.values.data.push_back(leftMotorSpeed);
		speed.values.data.push_back(righMotorSpeed);
		
		//publishing speed
		speedPub.publish(speed);
	

		rate.sleep();
	}
	
	
	ROS_INFO("leader_control_node just ended!\n");
	return(0);
}

