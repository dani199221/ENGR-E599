//
// Created by malintha on 11/29/17.
//

#ifndef PROJECT_GEOCONTROLLLERUTILS_H
#define PROJECT_GEOCONTROLLLERUTILS_H

#endif

#pragma once

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class geoControllerUtils {
public:
    Vector3d getVeeMap(Matrix3d mat) {
        Vector3d vee_vec;
        vee_vec[0] = mat(2,1);
        vee_vec[1] = mat(0,2);
        vee_vec[2] = mat(1,0);
        return vee_vec;
    }

    Matrix3d getSkewSymmetricMap(Vector3d vec) {
        Matrix3d hat_map(3,3);
        hat_map <<  0, -vec(3), vec(2),
                vec(3), 0, -vec(1),
                -vec(2), vec(1), 0;
        return hat_map;
    }
};
