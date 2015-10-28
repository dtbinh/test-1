#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "../include/v_repConst.h"
#include <math.h>
//ROS messeges
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_datatypes.h>

//V-REP includes
#include "vrep_common/VrepInfo.h"



// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
geometry_msgs::PoseStamped currentPose;

double R = 0.3, L = 1.0;
double firstline_x = 2*R;
double secondline_x = 0.0;
double startpoint_y = 0.0;
double XP, YP;
const double pi = 3.14159265359;

double mcp;
double mtg;
double thetap;
double el, ea;
double yaw, pitch,roll;



double distA, distB, distC, distD;

//Publisher
ros::Publisher errorPub;

template <class T> const T& min (const T& a, const T& b) {
  return !(b<a)?a:b;     // or: return !comp(b,a)?a:b; for version (2)
}

// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}



void poseCallback(const geometry_msgs::PoseStamped _pose){  

	
    	//(rand()%20-10)/100 - noise between -0.1 to 0.1 m in position and radians in orientation

		currentPose.pose.position.x = _pose.pose.position.x+(rand()%20-10)/100;
    	currentPose.pose.position.y = _pose.pose.position.y+(rand()%20-10)/100;
    	currentPose.pose.position.z = _pose.pose.position.z+(rand()%20-10)/100;
		double x,y,z,w;
		currentPose.pose.orientation.x = _pose.pose.orientation.x+(rand()%20-10)/100;
    	currentPose.pose.orientation.y = _pose.pose.orientation.y+(rand()%20-10)/100;
    	currentPose.pose.orientation.z = _pose.pose.orientation.z+(rand()%20-10)/100;
		currentPose.pose.orientation.w = _pose.pose.orientation.w+(rand()%20-10)/100;
		x = currentPose.pose.orientation.x;
		y = currentPose.pose.orientation.y;
		z = currentPose.pose.orientation.z;
		w = currentPose.pose.orientation.w;
		//yaw = atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);

tf::Quaternion q(x, y, z, w);
tf::Matrix3x3 m(q);
m.getRPY(roll, pitch, yaw);
}

// Main code:
int main(int argc,char* argv[])
{



	// Create a ROS node. The name has a random component: 
	int _argc = 0;
	char** _argv = NULL;
	struct timeval tv;
	unsigned int timeVal=0;
	if (gettimeofday(&tv,NULL)==0)
		timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
		
	ros::init(argc,argv,"error_node");
	ros::NodeHandle node("~");

	//relative topic speed



	


	if(!ros::master::check())
		return(0);
	
	ROS_INFO("error_node just started");

	// 1. Let's subscribe to V-REP's info stream 
	ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);

	// 3. Subscribe to pose of the robot, here it can be absolute or relative
	ros::Subscriber subPose=node.subscribe("/turtlebotPose",1,poseCallback);
	

	//publisher
	errorPub=node.advertise<std_msgs::Float32MultiArray>("/error",1);

	ros::Rate rate(10);
    //ROS_INFO("SPINNING @ 10Hz");

	while (ros::ok()){

			ros::spinOnce();
	
			//A: the first line

			if((currentPose.pose.position.y<=L) && (currentPose.pose.position.y>startpoint_y) && (currentPose.pose.position.x>R)) {

				el = currentPose.pose.position.x - firstline_x;
				XP = firstline_x;
				YP = currentPose.pose.position.y;
				//ROS_INFO("CASE: A");
			}


			//B: half circle

			else if( currentPose.pose.position.y>L) {
				el = sqrt( pow(R-currentPose.pose.position.x,2)+pow(L-currentPose.pose.position.y,2) )-R; //computation of the euclidian distans to the circle
               
				//-------------computation----------------
				double m = (currentPose.pose.position.x - R)/(currentPose.pose.position.y-L);
				double alpha;

				if(atan(m)>0){
					alpha = atan(m);
				}
				else if(atan(m)<0) {
					alpha = pi+atan(m);
				}
				else if(atan(m) == 0) {
					alpha = pi/2;
				}

				XP = R + R*cos(alpha);
				YP = L + R*sin(alpha);
				//----------------end-----------------------
				//ROS_INFO("CASE: B");
			}


			//C: the second line

			else if((currentPose.pose.position.y<=L) && (currentPose.pose.position.y>startpoint_y) && (currentPose.pose.position.x<=R)) {
	
				el =  secondline_x - currentPose.pose.position.x;
				XP = secondline_x;
				YP = currentPose.pose.position.y;
				//ROS_INFO("CASE: C");
			}
			
			//D: below the startpoint_y

			else if(currentPose.pose.position.y<=startpoint_y) {

				//el = sqrt( pow(firstline_x-currentPose.pose.position.x,2)+pow(startpoint_y-currentPose.pose.position.y,2) ); 
				el = currentPose.pose.position.x - firstline_x;
				//computation of the euclidian distans to the start point
				XP = firstline_x;
				YP = startpoint_y;
				//ROS_INFO("CASE: D x: %lf y: %lf" ,currentPose.pose.position.x,currentPose.pose.position.y);
			}


				//The idea of absolut value of lateral error is wrong

			//TODO distance sign is depend on orientation of the robot
			/*
			 if(yaw < 0 && YP<=L){
					el = -el;
				}
*/
			mcp = (XP - R)/(YP - L);
			mtg = - 1/mcp;
			thetap = pi + atan(mtg);

			if(XP == 2*R ){ //first line
				ea =  yaw - (pi/2); // TODO check the angle and wheel directions
//ROS_INFO("first line:  %lf",ea);
			}else if(XP == 0 ){ //second line
				ea = yaw + (pi/2);
//ROS_INFO("second line:  %lf",ea);
			}else{ //circle
				ea = yaw - thetap;
//ROS_INFO("circle:  e: %lf theta: %lf",ea, thetap);
			}
ROS_INFO("thetep %lf  ea: %lf yaw: %lf", thetap,ea,yaw);

//ea normalization
if(ea > 2*pi)
 ea = ea - 2*pi;
//double eaDeg = ea*180/pi;
//ROS_INFO("ea deg: %lf",eaDeg);

			std_msgs::Float32MultiArray error;
			error.data.push_back(el);
			error.data.push_back(ea);
			errorPub.publish(error);

			rate.sleep();
				
	}


    
    
		
	ROS_INFO("error_node just ended!\n");
	return(0);
}
