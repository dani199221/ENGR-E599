//
// Created by malintha on 11/27/17.
//

#ifndef PROJECT_TRACKING_CONTROL_H
#define PROJECT_TRACKING_CONTROL_H

#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "geoControlllerUtils.h"

using namespace Eigen;

class ControllerImpl {

public:
    ControllerImpl(ros::NodeHandle &nodeHandle): n(nodeHandle) {

        prev_R_d << 0,0,0,
                    0,0,0,
                    0,0,0;
        prev_Omega_d << 0,0,0;

        geoControllerUtils utils;

        e1[0] = e2[1] = e3[2] = 1;
        e1[1] = e1[2] = e2[0] = e2[2] = e3[0]= e3[1] = 1;

        // set values from the yaml file
        J[0] = utils.get(n, "uav/J/J1");
        J[1] = utils.get(n, "uav/J/J2");
        J[2] = utils.get(n, "uav/J/J3");
        m = utils.get(n, "uav/m");
        d = utils.get(n, "uav/d");
        ctf = utils.get(n, "uav/ctf");
        kx = utils.get(n, "controller/kx");
        kv = utils.get(n, "controller/kv");
        kr = utils.get(n, "controller/kr");
        kOmega = utils.get(n, "controller/kOmega");

        x0 = utils.getX0();
        v0 = utils.getV0();
        R0 = utils.getR0();
        Omega0 = utils.getOmega0();

    }

    void setDynamicsValues(Vector3d x, Vector3d v, Matrix3d R, Vector3d Omega) {
        if(isFirst) {
            this->x = x0;
            this->v = v0;
            this->R = R0;
            this->Omega = Omega0;
        }
        this->x = x;
        this->v = v;
        this->R = R;
        this->Omega = Omega;
    }

    /**
     * should be invoked after dynamic values are set
     * @param e
     * @return
     */
    Vector3d getForceVector(const ros::TimerEvent& e) {
        this->t = e.current_real.sec;
        this->dt = t - e.last_real.sec;

        calculate_x_desired();
        calculate_Rd();
        calculate_Omega_desired();
        calculateErrors();
        Vector3d f_temp = -(-kx*ex - kv*ev - m*g*e3 + m*xddot_d);
        Vector3d Re3 = R*e3;
        f = f_temp.cwiseProduct(Re3);
        return f;
    }

    Vector3d getMomentVector() {
        Matrix3d Omega_hat = utils.getSkewSymmetricMap(Omega);
        M = -kr*eR - kOmega*eOmega + Omega.cross(J.cwiseProduct(Omega)) -
                J.cwiseProduct((Omega_hat*Eigen::Transpose(R)*R_d)*Omega_d -
                                       (Eigen::Transpose(R)*R_d)*Omega_dot_d);
        return M;
    }

    void calculateErrors() {
        ex = x-x_d;
        ev = v - xdot_d;
        Matrix3d eR_temp = 0.5*(Eigen::Transpose(R_d)*R - Eigen::Transpose(R)*R_d);
        Vector3d eR = utils.getVeeMap(eR_temp);
        eOmega = Omega - Eigen::Transpose(R)*R_d*Omega_d;
    }


    void calculate_Rd() {
        Vector3d b3_d_nume = -kx*ex - kv*ev - m*g*e3 + m*xddot_d;
        b3_d = b3_d_nume/b3_d_nume.norm();
        b1_d << cos(M_PI*t), sin(M_PI*t), 0;
        Vector3d b2_d_nume = b3_d.cross(b1_d);
        b2_d = b2_d_nume/b2_d_nume.norm();
        R_d << b2_d.cross(b3_d), b2_d, b3_d;
        R_dot_d = (R_d - prev_R_d)/dt;

    }

    void calculate_Omega_desired() {
        Omega_d = utils.getVeeMap(R_dot_d*Eigen::Inverse(prev_R_d));
        Omega_dot_d = (Omega_d - prev_Omega_d)/dt;
        prev_Omega_d = Omega_d;
    }

    void calculate_x_desired() {
        x_d[0] = 0.4*t;
        x_d[1] = 0.4*sin(M_PI*t);
        x_d[2] = 0.6*cos(M_PI*t);

        xdot_d[0] = 0.4;
        xdot_d[1] = 0.4*cos(M_PI*t);
        xdot_d[2] = -0.6*sin(M_PI*t);

        xddot_d[0] = 0;
        xddot_d[1] = -0.4*sin(M_PI*t);
        xddot_d[2] = -0.6*cos(M_PI*t);
    }


private:
    ros::NodeHandle n;
    float dt;
    float t;

    // desired parameters
    Vector3d x_d;
    Vector3d xdot_d;
    Vector3d xddot_d;
    Vector3d b1_d;
    Vector3d b2_d;
    Vector3d b3_d;
    Matrix3d R_d;
    Matrix3d R_dot_d;
    Vector3d Omega_d;
    Vector3d Omega_dot_d;

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
    Vector3d Omega0;

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

    // translation vectors
    Vector3d e1;
    Vector3d e2;
    Vector3d e3;

    // force and moment vectors
    Vector3d f;
    Vector3d M;

    // other variables
    Matrix3d prev_R_d;
    Vector3d prev_Omega_d;
    bool isFirst = true;
    geoControllerUtils utils;

};