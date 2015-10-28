#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>


double V,omega;
bool twistFlag = false;
double xScan;
double yScan;
double zScan;

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_node");

  ros::NodeHandle n("~");

  n.param("xScan",xScan,0.0);
  n.param("yScan",yScan,0.0);
  n.param("zScan",zScan,0.0);

  tf::Transform transformBaseScan;
  tf::TransformBroadcaster br;
  ros::Rate r(100.0);
  while(n.ok()){

    ros::spinOnce(); 

	transformBaseScan.setOrigin( tf::Vector3( xScan,yScan,zScan) );
	tf::Quaternion q2;
   	q2.setRPY(0.0, 0.0, 0.0);
   	transformBaseScan.setRotation(q2);
   	br.sendTransform(tf::StampedTransform(transformBaseScan, ros::Time::now(), "/laser", "/base_link"));


    r.sleep();
  }
}
