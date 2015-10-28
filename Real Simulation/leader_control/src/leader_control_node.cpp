//ROS
#include <ros/ros.h>

//ROS msg
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8MultiArray.h>

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
	
	/* ***************
	   *SUBSCRIPTIONS*
	   *************** */
	
	// 1. Subscribe to the twist
	ros::Subscriber subDir=node.subscribe("/leader_velocity",1,velocityCallback);

	/* ************
	   *PUBLISHING*
	   ************ */
	
	// 1. Speed of the wheels of the leader
	ros::Publisher speedPub=node.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	
	ros::Rate rate(100);
	while (ros::ok()){
		
		ros::spinOnce();
				
		geometry_msgs::Twist vel;
		
		vel = velocity;
				
		//publishing speed
		speedPub.publish(vel);
	

		rate.sleep();
	}
	
	
	ROS_INFO("leader_control_node just ended!\n");
	return(0);
}

