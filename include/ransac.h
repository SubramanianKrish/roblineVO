/*

Defines ransac parameters and scoring functions for support evaluation

Date: 28 Nov 2020

Changelog:
    subbu - 11/28 - Initial commit

*/
#pragma once

#include <Eigen/Dense>

typedef Eigen::Vector3d point3d;
typedef Eigen::Matrix3d covariance;

class Ransac{

public:
    double mahalanobis_distance(const point3d& X, const covariance& sigma_x, const point3d& A, const point3d& B);
};
