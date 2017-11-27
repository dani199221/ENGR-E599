//
// Created by malintha on 11/27/17.
//

#ifndef PROJECT_TRACKING_CONTROL_H
#define PROJECT_TRACKING_CONTROL_H

#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

using namespace Eigen;

double get(
        const ros::NodeHandle& n, const std::string& name) {
        double value;
        n.getParam(name, value);
        return value;
}

class ControllerImpl {

public:
    ControllerImpl(ros::NodeHandle &nodeHandle): n(nodeHandle) {
        J[0] = get(n, "uav/J/J1");
        J[1] = get(n, "uav/J/J2");
        J[2] = get(n, "uav/J/J3");
        m = get(n, "uav/m");
        d = get(n, "uav/d");
        ctf = get(n, "uav/ctf");
        kx = get(n, "controller/kx");
        kv = get(n, "controller/kv");
        kr = get(n, "controller/kr");
        kOmega = get(n, "controller/kOmega");





    }

    void setDynamicsValues(Vector3d x, Vector3d v, Matrix3d R, Vector3d Omega, Vector3d Omega_dot) {
        this->x = x;
        this->v = v;
        this->R = R;
        this->Omega = Omega;
        this->Omega_dot = Omega_dot;
    }

    void calculateErrors() {
        ex = x-x_d;
        ev = v - xdot_d;
        Matrix3d eR_temp = 0.5*(Eigen::Transpose(R_d)*R - Eigen::Transpose(R)*R_d);


    }

    Vector3d get_b1d_t(float t) {
        b1_d[0] = cos(M_PI*t);
        b1_d[1] = sin(M_PI*t);
        b1_d[2] = 0;
        return b1_d;
    }

    Vector3d get_x_d(float t) {
        x_d[0] = 0.4*t;
        x_d[1] = 0.4*sin(M_PI*t);
        x_d[2] = 0.6*cos(M_PI*t);
        return x_d;
    }

    Vector3d get_xdot_d(float t) {
        xdot_d[0] = 0.4;
        xdot_d[1] = 0.4*cos(M_PI*t);
        xdot_d[2] = -0.6*sin(M_PI*t);
        return xdot_d;
    }

    Vector3d get_xddot_d(float t) {
        xddot_d[0] = 0;
        xddot_d[1] = -0.4*sin(M_PI*t);
        xddot_d[2] = -0.6*cos(M_PI*t);
        return xddot_d;
    }

    void setdt (float dt) {
        this->dt = dt;
    }

    Vector3d getTheVeeMap(Matrix3d mat) {
        Vector3d vee_vec;
        vee_vec[0] = mat(2,1);
        vee_vec[1] = mat(0,2);
        vee_vec[2] = mat(1,0);
        return vee_vec;
    }

private:
    ros::NodeHandle n;
    float dt;

    // desired parameters
    Vector3d x_d;
    Vector3d xdot_d;
    Vector3d xddot_d;
    Vector3d b1_d;
    Vector3d b3_d;
    Matrix3d R_d;
    Vector3d Omega_d;
    Vector3d Omegad_d;

    // actual parameters
    Vector3d x;
    Vector3d v;
    Matrix3d R;
    Vector3d Omega;
    Vector3d Omega_dot;

    // initial parameters
    Vector3d x0;
    Vector3d v0;
    Matrix3d R0;

    // error vectors
    Vector3d ex;
    Vector3d ev;
    Vector3d eR;
    Vector3d eOmega;

    // drone specific and controller specific values
    Vector3d J;
    float m;
    float d;
    float ctf;
    float kx;
    float kv;
    float kr;
    float kOmega;
    const float g = 9.8;






};