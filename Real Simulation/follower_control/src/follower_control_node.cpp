//ROS
#include <ros/ros.h>

//ROS msg
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

// Global variables (modified by topic subscribers):

double sensor_distance;
double VLead;
double el;
double ea;

		
//Control
bool errorFlag = false;
bool sensorFlag = false;



//ABSOLUTE VALUE FUNCTION FOR DOUBLE
double abs2(double a){
	if(a<0)a=-a;
	return a;
}


// Topic subscriber callbacks:

void errorCallback(const std_msgs::Float32MultiArray errors){  
    el = errors.data[0];
	ea = errors.data[1];
	errorFlag = true;
}

void sensorCallback(const std_msgs::Float64 sens){
	sensor_distance = sens.data;
	sensorFlag = true;

}

void velocityCallback(const geometry_msgs::Twist _velocity){
	VLead = _velocity.linear.x;
}



int main(int argc,char* argv[])
{
	ros::init(argc,argv,"turtle_leader_node");
	ros::NodeHandle node("~");

	if(!ros::master::check()) return(0);
	
	ROS_INFO("follower_control_node just started");

	//Precedent angular and linear velocity
	double PrecV;
	double PrecO;
	
	//Maximum angular and linear allowed velocity
	const double VMAX = 0.3;
	const double OMAX = 1.5;
	
	//Linear velocity control parameters
	double Kprop = 1;
	double Kinte = 0.01;
	const double h = 0.3;
	
	const double rangeDistMin = 0.25;
	double rangeDistErr = 0;
	double integrall = 0;
	
	//Angular velocity control parameters
	double Kp,Kd;
			
	//Parameters
	node.param("Kp", Kp, 1.0);
 	node.param("Kd", Kd, 1.0);
  	
	
	/* ***************
	   *SUBSCRIPTIONS*
	   *************** */
	
	// 1. Subscribe to lateral and angular errors
	ros::Subscriber errors = node.subscribe<std_msgs::Float32MultiArray> ("/error",1,errorCallback);
	// 2. Subscribe to Sensor 
	ros::Subscriber sub=node.subscribe<std_msgs::Float64>("/sensor",1,sensorCallback);
	// 3. Subscribe to the velocity of leader
	ros::Subscriber subDir=node.subscribe("/leader_velocity",1,velocityCallback);
	
	/* ************
	   *PUBLISHING*
	   ************ */
	
	// 1. Speed of the wheels of the follower
	ros::Publisher speedPub=node.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	
	int rateFreq = 100;
	ros::Rate rate(rateFreq);

	while (ros::ok()){
		ros::spinOnce();

	
		double V,omega;

				
		if(errorFlag){
			
         	//------Linear velocity controller (PI)------
			if(sensorFlag){
				double rangeDist = sensor_distance;
				double rangeDistDesir = rangeDistMin + h*VLead;		//Control law
	
				rangeDistErr = rangeDist - rangeDistDesir; 			//Proportional part
				integrall = integrall + rangeDistErr*(1/rateFreq); 	//Integral part
				
				V = Kprop*rangeDistErr + Kinte*integrall; 
				
				//Bound of the linear velocity
				if(V > VMAX){
					V = VMAX;
				}else if(V< 0){
					V = 0;
				}
			}else{
				V = VMAX;
			}
			
			// ------end of velocity controller------


			//------ Angular velocity controller------
			double part = V*pow(cos(ea),3);
			double omega_kp = -part*Kp*el;
			double omega_kd = -abs2(part)*Kd*tan(ea);
			omega = omega_kp + omega_kd;
						
			if(omega > OMAX){
				omega = OMAX;
			}else if(omega < -OMAX){
				omega = -OMAX;
			}
			// ------end of velocity controller------
			
			PrecV = V;
			PrecO = omega;
			
			errorFlag = false;
		}else{
			V = PrecV;
			omega = PrecO;
		}
		
		
		
		geometry_msgs::Twist vel;
		
		vel.linear.x = V;
		vel.linear.y = 0;
		vel.linear.z = 0;
		
		vel.angular.x = 0;
		vel.angular.y = 0;
		vel.angular.z = omega;
		
		//publishing speed
		speedPub.publish(veò);
	
			
    	rate.sleep();
	}
	
	
	ROS_INFO("follower_control_node just ended!\n");
	return(0);
}
