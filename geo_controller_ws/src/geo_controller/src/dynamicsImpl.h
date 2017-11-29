//
// Created by malintha on 11/28/17.
//

#ifndef PROJECT_DYNAMICSIMPL_H
#define PROJECT_DYNAMICSIMPL_H
#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

using namespace Eigen;

class dynamicsImpl{
public:
    float temp_acc;
    dynamicsImpl(ros::NodeHandle &n):nodeHandle(n) {
        n.getParam("/cf/crazyflie_add/tf_prefix", tf_prefix);
        m_subscribeImuMsgs = n.subscribe("/crazyflie/imu", 1, &dynamicsImpl::ImuValRecieved, this);
    }

    void ImuValRecieved(const sensor_msgs::Imu::ConstPtr& msg) {

        temp_acc = msg->angular_velocity.x;
        Omega << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
        //angular_velocity.y => forward (pitch), x => roll

    }

    Vector3d getOmega() {
        return Omega;
    }


private:
    ros::NodeHandle &nodeHandle;
    ros::Subscriber m_subscribeImuMsgs;
    std::string tf_prefix;
    Vector3d Omega;
    Vector3d x_dot;

};

