//
// Created by malintha on 11/27/17.
//

#ifndef PROJECT_TRACKING_CONTROL_H
#define PROJECT_TRACKING_CONTROL_H

#endif

#pragma once

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class ControllerImpl {

public:
    ControllerImpl():xd_t(0,0,0), b1d_t(0,0,0) {
        /**
         * read the file and set the constant values
         * J = [0.0820, 0.0845, 0.1377] kgm^2
         * m
         * d
         * ctf
         * kx
         * kv
         * kR
         * kOmega
         *
         */

    }

    Vector3d get_b1d_t(float t) {
        b1d_t[0] = cos(M_PI*t)
        b1d_t[1] = sin(M_PI*t);
        b1d_t[2] = 0;
        return b1d_t;
    }

    Vector3d get_xd_t(float t) {
        xd_t[0] = 0.4*t;
        xd_t[1] = 0.4*sin(M_PI*t);
        xd_t[2] = 0.6*cos(M_PI*t);
        return xd_t;
    }



private:
    Vector3d xd_t; // get the time in secs (0.001, 0.002..) to get x positions.
    Vector3d b1d_t;
    Vector3d b3d_t;

    Vector3d x_t;
    Vector3d xdot_t;
    Vector3d xddot_t;

    const float g = 9.8;

    float dt;



};