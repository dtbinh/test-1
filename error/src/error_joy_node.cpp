/**
 * \file error_joy_node.cpp
 * \brief A node to compute the follower angular and lateral error with respect to the trajectory of the leader.
 * \author Alessandro Carfì and Rafał Kosk
 * \version 0.1
 * \date 23 June 2015
 * 
 * 
 * Subscribes to: <BR>
 *    ° Absolute topic "/turtlebotPose" <BR>
 *    ° Absolute topic "/turtlebotPose_1" <BR>
 * 
 * Publishes to: <BR>
 *    ° Absolute topic "/error_joy"
 *    ° Relative topic "visualization_marker"
 *
 * This ROS node is used to compute lateral and angular error for robots performing
 * a platoon formation, it publish also markers that can be used from rviz to display the trajectory
 *
 */


//ROS
#include <ros/ros.h>

#include "../include/spline.h"

//ROS messeges
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

// Global variables :
bool leaderPoseFlag = false;
double leaderX, leaderY;
double followerX, followerY;
double yaw, pitch,roll;

struct point_{
    double x;
    double y;
	double dx;
	double dy;
};


bool Nearest(double *, double *, double *, double *, point_ *);

//ABSOLUTE VALUE FUNCTION FOR DOUBLE
double abs2(double a){
	if(a<0)a=-a;
	return a;
}

void splineCreate(tk::spline &spl,std::vector<double> &_timeVect, std::vector<double> &_Points){

	spl.set_points(_timeVect,_Points);

//result of the spline: y = spl(x)

}

// Topic subscriber callbacks:

void poseCallback(const geometry_msgs::PoseStamped _pose){ 
    	
		followerX = _pose.pose.position.x;
    	followerY = _pose.pose.position.y;
    	
		double x,y,z,w;
		
		x = _pose.pose.orientation.x;
		y = _pose.pose.orientation.y;
		z = _pose.pose.orientation.z;
		w = _pose.pose.orientation.w;
		
		tf::Quaternion q(x, y, z, w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
}

void poseLeadCallback(const geometry_msgs::PoseStamped _pose){  

    	leaderX = _pose.pose.position.x;
		leaderY = _pose.pose.position.y;	

		leaderPoseFlag = true;
}




// Main code:
int main(int argc,char* argv[])
{

	//ROS initialization:
			
	ros::init(argc,argv,"error_joy_node");
	ros::NodeHandle node("~");
	
	if(!ros::master::check()) return(0);
	
	ROS_INFO("error_joy_node just started");

	 
	/* ***************
	   *SUBSCRIPTIONS*
	   *************** */
		
	// 1. Subscribe to pose of the follower robot
	ros::Subscriber subPose=node.subscribe("/turtlebotPose",1,poseCallback);
	// 2. Subscribe to pose of the leader robot
	ros::Subscriber subPoseLead=node.subscribe("/turtlebotPose_1",1,poseLeadCallback);
	
	/* ************
	   *PUBLISHING*
	   ************ */

	// 1. Publish angular and lateral angular error
	ros::Publisher errorPub=node.advertise<std_msgs::Float32MultiArray>("/error_joy",1);
	// 2. Publish marker in order to be able to plot spline on rviz
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	
	std::vector<double> X, Y, timeVect;
	
	
	ros::Rate rate(20);
    //ROS_INFO("SPINNING @ 20Hz");

	while (ros::ok()){

			//----BEGIN: Marker points initialization
			visualization_msgs::Marker points;
			points.header.frame_id = "/world";
			points.header.stamp = ros::Time::now();
		 	points.ns = "points";
			points.action = visualization_msgs::Marker::ADD;
			points.pose.orientation.w = 1.0;
			points.id = 0;
			points.type = visualization_msgs::Marker::POINTS;
			// POINTS markers use x and y scale for width/height respectively
			points.scale.x = 0.05;
			points.scale.y = 0.05;
			// Points are green
			points.color.g = 1.0f;
			points.color.a = 1.0;
			//----END
		
			
		    //Variables
		    bool splineFlag=false;
			tk::spline splX,splY;
			
			ros::spinOnce();
			
		    
		
		    if(leaderPoseFlag){
						
				//Strage of Leader last 200 positions
				if(X.size() == 0){
					X.push_back(leaderX);
					Y.push_back(leaderY);
					timeVect.push_back(ros::Time::now().toSec());
				}else{				
					double distance = sqrt(pow(X.back()-leaderX,2)+pow(Y.back()-leaderY,2));
					
					if(distance > 0.01){
						X.push_back(leaderX);
						Y.push_back(leaderY);
						timeVect.push_back(ros::Time::now().toSec());
                    }
                    
                    if( X.size()>100){
                        X.erase(X.begin());
						Y.erase(Y.begin());
						timeVect.erase(timeVect.begin());
                    }
				}
				
				//Creation of the Spline
				if(X.size()>3) {
                    splineCreate(splX,timeVect,X);
					splineCreate(splY,timeVect,Y);
					splineFlag = true;
				}



				//Computation of el-ea
				if(splineFlag){
					std::vector<point_> ps;
					point_ Near;
					double alpha;
					double ea, el;
					double t = timeVect[0];
					double sample = 0.1;
                    //Search on the spline the nearest point to the actual position of the follower
					while( t < (timeVect.back()- sample)){
						geometry_msgs::Point p;
						
						double X1 = splX(t);
						double Y1 = splY(t);
				
						double X2 = splX(t+0.5);
						double Y2 = splY(t+0.5);
						
                        //Fit the marker to plot the spline
                       	p.x = X1;
						p.y = Y1;
						p.z = 0;
						points.points.push_back(p);
						
						if(Nearest(&X1, &Y1, &X2, &Y2, &Near)){
							ps.push_back(Near);
						}
						
						t += sample; 
					}
					
                    //Publish points of spline
					marker_pub.publish(points);
					
					if(ps.size() == 0){
						Near.x = splX(timeVect[0]);
						Near.y = splY(timeVect[0]);
						Near.dx = (splY(timeVect[1])-Near.x);
						Near.dy = (splY(timeVect[1])-Near.y);
					}else{
                        
						double dist = std::numeric_limits<double>::infinity(); //It sets dist= INF;
                        
                        for (std::vector<point_>::iterator it = ps.begin() ; it != ps.end(); ++it){
                            double tempD = sqrt(pow(it->x-followerX,2) + pow(it->y-followerY,2));
							if(dist > tempD){
								dist = tempD;
								Near.x = it->x;
								Near.y = it->y;
								Near.dx = it->dx;
								Near.dy = it->dy;
							}
                        } 
					}
					
					/*For convention we are working only with positive angles [0;2PI],
                      then every time that we found a negavie angle we convert it*/
                    
					alpha = atan2(Near.dy,Near.dx);
					
					if(yaw<0) yaw = yaw + 2*M_PI;
					
			    	if(alpha < 0) alpha += 2*M_PI;
					
					ea = yaw - alpha ;
					
					// 0-2PI singularity
					if(ea > (M_PI)){
						ea -= 2*M_PI;
					} else if(ea < -(M_PI)){
						ea += 2*M_PI;
					}
					
                    el = -sin(alpha)*followerX + cos(alpha)*followerY -cos(alpha)*Near.y + sin(alpha)*Near.x;	
					
					std_msgs::Float32MultiArray error;
					error.data.push_back(el);
					error.data.push_back(ea);
					errorPub.publish(error);
			    					
				}//end of splineFlag
			
				leaderPoseFlag = false;
			}
			
			rate.sleep();	
	}

    
    ROS_INFO("error_joy_node just ended!\n");
	return(0);
}


//This function provides x and y of one possible near poin (for more informations check the paper)
bool Nearest(double *x1, double *y1, double *x2, double *y2, point_ *projection){
	
	double dx = *x2 - *x1;
	double dy = *y2 - *y1;
	double m = dy/dx;
	
	projection->x =  (m/(pow(m,2)+1))*((m*(*x1)-(*y1))+(followerX/m +followerY));
	projection->y = -(1/(pow(m,2)+1))*((m*(*x1)-(*y1))+(followerX/m+followerY))+(followerX/m+followerY);
	
	if((projection->x >= *x1 && projection->x <= *x2) || (projection->x <= *x1 && projection->x >= *x2)){
		projection->x = *x1;
        projection->y = *y1;
		projection->dx = dx;
		projection->dy = dy;
        return true;
	}else{	
		return false;
	}
}