/**
 * \file scanner_node.cpp
 * \brief A node that reads the stream of a laser rangefinder and returns the minimum value.
 * \author Alessandro Carfì and Rafał Kosk
 * \version 0.1
 * \date 23 June 2015
 * 
 *
 * Subscribes to: <BR>
 *    ° Absolute topic "/laser_finder" <BR>
 * 
 * Publishes to: <BR>
 *    ° Absolute topic "/distance"
 *
 * This ROS node is used to find the minimum distance inside a stream of data coming from a laser rangefinder sensor
 * in a specific radius range [pi/6;4/6 pi].
 *
 */



#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <math.h>

bool laserFlag = false;
double distance;

void laserCallback(const sensor_msgs::LaserScan data)
{
	double increment = data.angle_increment;
	double actual_angle = 0;
	double range = std::numeric_limits<double>::infinity();
	
	for(int i=0; i<data.ranges.size(); i++){
		if(actual_angle > M_PI/6 || actual_angle < 4*M_PI/6){
			if(data.ranges[i] < range && data.ranges[i]!=0){
				range = data.ranges[i];
			}
		}
		actual_angle += increment;
	}
	
	if(range != std::numeric_limits<double>::infinity()){
		laserFlag = true;
		distance = range;
		ROS_INFO("range: %f",range);
	}
}

int main(int argc,char* argv[])
{
	ros::init(argc,argv,"scanner_node");
	ros::NodeHandle node("~");
	
	if(!ros::master::check())	return(0);
	
	/* **************
	   *SUBSCRITIONS*
	   ************** */
	
	//2. Subscribe to the laser range finder data stream
	ros::Subscriber errors = node.subscribe<sensor_msgs::LaserScan> ("/laser_finder",1,laserCallback);

	/* ************
	   *PUBLISHING*
	   ************ */
	
	//1. Pulish minimum distance in the intervall
	ros::Publisher pub = node.advertise<std_msgs::Float64>("/distance", 1);

	ros::Rate rate(100);
	
	while (ros::ok()){
		ros::spinOnce();
		if(laserFlag){
			std_msgs::Float64 d;
			d.data = distance;
			pub.publish(d);
			laserFlag = false;
		}
		rate.sleep();
	}
}