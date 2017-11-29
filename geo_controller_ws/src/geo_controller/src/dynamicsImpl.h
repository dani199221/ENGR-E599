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
#include <tf/transform_listener.h>

using namespace Eigen;

class dynamicsImpl {
public:
    float temp_acc;

    dynamicsImpl(ros::NodeHandle &n, std::string worldFrame, std::string bodyFrame) : nodeHandle(n),
                                                                                      worldFrame(worldFrame),
                                                                                      bodyFrame(bodyFrame) {
        n.getParam("/cf/crazyflie_add/tf_prefix", tf_prefix);

        ROS_INFO("### Initialized dynamicsImpl worldFrame:%s bodyFrame%s\n",worldFrame.data(), bodyFrame.data());

        transformListener.waitForTransform(worldFrame, bodyFrame, ros::Time(0), ros::Duration(10.0));
        m_subscribeImuMsgs = n.subscribe("/crazyflie/imu", 1, &dynamicsImpl::ImuValRecieved, this);
    }

    void ImuValRecieved(const sensor_msgs::Imu::ConstPtr &msg) {
        x_ddot << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        Omega << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        //angular_velocity.y => forward (pitch), x => roll
    }

    /**
     * set dt before executing any other function
     * @param dt
     */
    void setdt(float dt) {
        this->dt = dt;
    }

    Vector3d getOmega() {
        return Omega;
    }


    Vector3d get_x_ddot() {
        return x_ddot;
    }

    Vector3d* get_x_v_Omega() {
        tf::StampedTransform transform;
        transformListener.lookupTransform(worldFrame, bodyFrame, ros::Time(0), transform);

        x << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        x_dot = (x - prev_x)/dt;


        x_arr[0] = x;
        x_arr[1] = x_dot;
        x_arr[2] = Omega;

        prev_x = x;
        prev_x_dot = x_dot;
        return x_arr;
    }



private:
    ros::NodeHandle &nodeHandle;
    ros::Subscriber m_subscribeImuMsgs;
    tf::TransformListener transformListener;
    ros::Publisher m_pubNav;

    std::string worldFrame;
    std::string bodyFrame;

    float dt;

    std::string tf_prefix;
    Vector3d Omega;
    Vector3d x_ddot;
    Vector3d x_dot;
    Vector3d x;
    Vector3d prev_x;
    Vector3d prev_x_dot;

    Vector3d x_arr[3];

};

