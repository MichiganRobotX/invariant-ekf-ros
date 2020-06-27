#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>
#include <fstream>


using namespace std;

string filepath_raw = "/home/chenli/vrx_ws/src/vrx/invariant-ekf-ros/inekf_ros/tests/result/data_odo_gps.csv";
string filepath_filtered = "/home/chenli/vrx_ws/src/vrx/invariant-ekf-ros/inekf_ros/tests/result/data_filtered_gps.csv";
ofstream file;

void rawCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  //ROS_INFO("odometry: [%f]", msg->pose.pose.position.x);
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double x = msg->latitude;
	double y = msg->longitude;
	double z = msg->altitude;
    cout << t1 << "," << x << "," << y << "," << z << endl;
	file.open(filepath_raw.c_str(), ios::app);
    file.precision(16);
	file << t1 << "," << x << "," << y << "," << z << endl;
	file.close();
}

void filterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  //ROS_INFO("filtered: [%f]", msg->pose.pose.position.x);
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double x = msg->latitude;
	double y = msg->longitude;
	double z = msg->altitude;
	file.open(filepath_filtered.c_str(), ios::app);
    file.precision(16);
	file << t1 << "," << x << "," << y << "," << z << endl;
	file.close();
}

int main(int argc, char **argv)
{
	file.open(filepath_raw.c_str());
	file << "timestamp [ns]" << "," << "raw lati" << "," << "raw longti" << "," << "raw alti" << endl;
	file.close();

	file.open(filepath_filtered.c_str());
	file << "timestamp [ns]" << "," << "filtered lati" << "," << "filtered longti" << "," << "filtered alti" << endl;
	file.close();

  ros::init(argc, argv, "listener_gps");
  ros::NodeHandle n_;

  ros::Subscriber raw_sub_ = n_.subscribe("/sensors/an_device/NavSatFix", 1000, rawCallback);
  ros::Subscriber filtered_sub_ = n_.subscribe("/wamv/robot_localization/gps/filtered", 1000, filterCallback);

	ros::Rate loop_rate(500);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}