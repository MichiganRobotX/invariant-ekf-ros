/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Measurement.h
 *  @author Ross Hartley
 *  @brief  Header file for Measurement class
 *  @date   September 27, 2018
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "gazebo_msgs/LinkStates.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"
#include "inekf_msgs/LandmarkArray.h"
#include "InEKF.h"
#include "tf/transform_listener.h"

enum MeasurementType {EMPTY, IMU, GPS, LINK, LANDMARK, KINEMATIC, CONTACT};


class Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Measurement();
        virtual ~Measurement() = default;

        double getTime();
        MeasurementType getType();

        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    protected:
        double t_;
        MeasurementType type_;
};

struct MeasurementCompare {
  bool operator()(const std::shared_ptr<Measurement> lhs, const std::shared_ptr<Measurement> rhs) const {
    return lhs->getTime() > rhs->getTime();
  }
};


class ImuMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
        Eigen::VectorXd getData();

    private: 
        Eigen::Matrix<double,6,1> data_;
};

class GpsMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GpsMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);
        Eigen::VectorXd getData();

    private: 
        Eigen::Matrix<double,3,1> data_;
};

class GtLinkMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GtLinkMeasurement(const gazebo_msgs::LinkStates::ConstPtr& msg, const double t);
        Eigen::VectorXd getPos();
        Eigen::VectorXd getOri();

    private: 
        Eigen::Matrix<double,3,1> pos_; // x y z
        Eigen::Matrix<double,4,1> ori_; // x y z w
};


class LandmarkMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        LandmarkMeasurement(const inekf_msgs::LandmarkArray::ConstPtr& msg, const tf::StampedTransform& transform);
        inekf::vectorLandmarks getData();

    private:
        inekf::vectorLandmarks data_;
};

class ContactMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ContactMeasurement(const inekf_msgs::ContactArray::ConstPtr& msg);
        std::vector<std::pair<int,bool>> getData();

    private:
        std::vector<std::pair<int,bool>> data_;
};


class KinematicMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        KinematicMeasurement(const inekf_msgs::KinematicsArray::ConstPtr& msg);
        inekf::vectorKinematics getData();

    private:
        inekf::vectorKinematics data_;
};


#endif 
