//
// Created by malintha on 11/29/17.
//

#ifndef PROJECT_GEOCONTROLLLERUTILS_H
#define PROJECT_GEOCONTROLLLERUTILS_H

#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/node_handle.h>

using namespace Eigen;



class geoControllerUtils {
public:
    geoControllerUtils() {}

    double get(const ros::NodeHandle &n, const std::string &name) {
        const std::string node_prefix = "/crazyflie/geocontroller/";
        std::string key = node_prefix+name;
        double value;
        n.getParam(key, value);
        return value;
    }

    void initializeMatrices(const ros::NodeHandle &n) {
        x0 << get(n, "trajectory/x0/x01"), get(n, "trajectory/x0/x02"), get(n, "trajectory/x0/x03");
        v0 << get(n, "trajectory/v0/v01"), get(n, "trajectory/v0/v02"), get(n, "trajectory/v0/v03");
        R0 << get(n, "trajectory/R0/R0_r1/r1_1"), get(n, "trajectory/R0/R0_r1/r1_2"), get(n, "trajectory/R0/R0_r1/r1_3"),
                get(n, "trajectory/R0/R0_r2/r2_1"), get(n, "trajectory/R0/R0_r2/r2_2"), get(n, "trajectory/R0/R0_r2/r2_3"),
                get(n, "trajectory/R0/R0_r3/r3_1"), get(n, "trajectory/R0/R0_r3/r3_2"), get(n, "trajectory/R0/R0_r3/r3_3");
        Omega0 << get(n, "trajectory/Omega0/Omega01"), get(n, "trajectory/Omega0/Omega02"), get(n, "trajectory/Omega0/Omega03");
    }

    Vector3d getVeeMap(Matrix3d mat) {
        Vector3d vee_vec;
        vee_vec[0] = mat(2, 1);
        vee_vec[1] = mat(0, 2);
        vee_vec[2] = mat(1, 0);
        return vee_vec;
    }

    Matrix3d getSkewSymmetricMap(Vector3d vec) {
        Matrix3d hat_map(3, 3);
        hat_map << 0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
        return hat_map;
    }

    Vector3d getOmega0() {
        return Omega0;
    }

    Matrix3d getR0() {
        return R0;
    }

    Vector3d getX0() {
        return x0;
    }
    Vector3d getV0() {
        return v0;
    }



private:
    Vector3d x0;
    Vector3d v0;
    Matrix3d R0;
    Vector3d Omega0;
};

