/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF_ROS.h
 *  @author Ross Hartley
 *  @brief  Header file for a ROS wrapper of the Invariant EKF 
 *  @date   September 27, 2018
 **/

#ifndef INEKF_ROS_H
#define INEKF_ROS_H 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>
#include <boost/lockfree/queue.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "InEKF.h"
#include "Measurement.h"
#include "Queue.h"
#include "inekf_msgs/State.h"
#include "visualization_msgs/MarkerArray.h"
//#include "apriltag_msgs/AprilTagDetectionArray.h"
#include <mutex>

#define QUEUE_BUFFER_SIZE 100
#define MAX_QUEUE_SIZE 150


class InEKF_ROS {
    public:
        InEKF_ROS(ros::NodeHandle n);
        void init();
        void run();

    private: 
        ros::NodeHandle n_;
        inekf::InEKF filter_;
        ros::Subscriber imu_sub_;
        ros::Subscriber gps_sub_;
        ros::Subscriber linkstates_sub_;
        ros::Subscriber landmarks_sub_;
        ros::Subscriber kinematics_sub_;
        ros::Subscriber contact_sub_;
        std::thread filtering_thread_;
        std::thread output_thread_;
        Queue<std::shared_ptr<Measurement>, std::vector<std::shared_ptr<Measurement>>, MeasurementCompare> m_queue_;

        std::string imu_frame_id_;
        std::string gps_frame_id_;
        std::string map_frame_id_;
        bool publish_visualization_markers_;
        ros::Publisher visualization_pub_;
        bool enable_landmarks_;
        tf::StampedTransform camera_to_imu_transform_;
        bool enable_kinematics_;
        bool enable_linkstates;

        Eigen::Vector3d initial_linkstate_;
        Eigen::Vector4d cur_baselink_orientation_;
        Eigen::Matrix3d initial_R_;
        tf::StampedTransform base_to_gps_transform_;
        Eigen::Matrix<double,3,1> initial_lla_;
        Eigen::Matrix<double,3,1> initial_ecef_;
        double initial_yaw_;
        Eigen::Vector3d Og0_to_Ob0_;
        std::ofstream file;
        std::string gps_file_path_;
        bool output_gps_;

        void subscribe();
        void mainFilteringThread();
        void outputPublishingThread();
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); 
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void linkstatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);  
        void landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg);
        //void aprilTagCallback(const apriltag_msgs::AprilTagDetectionArray::ConstPtr& msg);
        void kinematicsCallback(const inekf_msgs::KinematicsArray::ConstPtr& msg);
        void contactCallback(const inekf_msgs::ContactArray::ConstPtr& msg);

        Eigen::Vector3d lla_to_ecef(const Eigen::Matrix<double,3,1>& lla);
        Eigen::Matrix<double,3,1> lla_to_enu(const Eigen::Matrix<double,3,1>& lla);

        void publishLandmarkMeasurementMarkers(std::shared_ptr<LandmarkMeasurement> ptr);
        void publishKinematicMeasurementMarkers(std::shared_ptr<KinematicMeasurement> ptr);
};

#endif 
