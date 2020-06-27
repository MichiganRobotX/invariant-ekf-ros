#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>
#include <fstream>
#include <string>


using namespace std;

string filepath_odo = "/home/chenli/vrx_ws/src/vrx/invariant-ekf-ros/inekf_ros/tests/result/data_odo_xyz.csv";
string filepath_filtered = "/home/chenli/vrx_ws/src/vrx/invariant-ekf-ros/inekf_ros/tests/result/data_filtered_xyz.csv";
ofstream file;

void rawCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	int n = (msg->pose).size();
	double x = msg->pose[n-9].position.x;
	double y = msg->pose[n-9].position.y;
	double z = msg->pose[n-9].position.z;
	file.open(filepath_odo.c_str(), ios::app);
	file << 0 << "," << x << "," << y << "," << z << endl;
	file.close();
}

void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("odometry: [%f]", msg->pose.pose.position.x);
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	double z = msg->pose.pose.position.z;
	file.open(filepath_odo.c_str(), ios::app);
	file << t1 << "," << x << "," << y << "," << z << endl;
	file.close();
}

void filterCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  //ROS_INFO("filtered: [%f]", msg->pose.pose.position.x);
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	double z = msg->pose.pose.position.z;
	file.open(filepath_filtered.c_str(), ios::app);
	file.precision(16);
	file << t1 << "," << x << "," << y << "," << z << endl;
	file.close();
}

int main(int argc, char **argv)
{
	//file.open(filepath_odo.c_str());
	//file << "timestamp [ns]" << "," << "odo x" << "," << "odo y" << "," << "odo z" << endl;
	//file.close();

	file.open(filepath_filtered.c_str());
	file << "timestamp [ns]" << "," << "filtered x" << "," << "filtered y" << "," << "filtered z" << endl;
	file.close();

  ros::init(argc, argv, "listener_xyz");
  ros::NodeHandle n_;

  //ros::Subscriber odo_sub_ = n_.subscribe("/gazebo/link_states", 1000, rawCallback);
  ros::Subscriber filtered_sub_ = n_.subscribe("/wamv/robot_localization/odometry/filtered", 1000, filterCallback);

	ros::Rate loop_rate(500);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}