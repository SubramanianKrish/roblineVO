/*

Defines ransac parameters and scoring functions for support evaluation

Date: 28 Nov 2020

Changelog:
    subbu - 11/28 - Initial commit

*/
#pragma once

// #include "utils.h"
#include <Eigen/Dense>

typedef Eigen::Vector3d point3d;
typedef Eigen::Matrix3d covariance;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> points3d;

class Ransac{
public:
    
    int n_iterations;
    double threshold;

    std::random_device rd;   // hardware generated random number
    std::mt19937 generator;  // seeding

    Ransac(int n_iterations, double threshold);

    // Returns the mahalanobis distance between a point and a line
    double mahalanobis_distance(const point3d& X, const point3d& A, const point3d& B, const Eigen::Vector3d& D, const Eigen::Matrix3d& U);

    // Returns a set of 3D points that are considered inliers to the estimated line
    Eigen::Matrix3Xd removeOutlierPoints(const Eigen::Matrix3Xd& linepoints, const std::vector<Eigen::Vector3d>& eig_val,
                                         const std::vector<Eigen::Matrix3d>& eig_vector, std::vector<Eigen::Matrix3d>& updated_covariance,
                                         std::vector<Eigen::Matrix3d>& updated_inv_root_covariance, const std::vector<Eigen::Matrix3d>& cov_G,
                                         const std::vector<Eigen::Matrix3d>& inv_cov_G);

    std::vector<int> ransac3D(const std::vector<points3d>& optLines1, const std::vector<points3d>& optLines2, const std::vector<std::vector<Eigen::Matrix3d>>& endPtCov1,
                             const std::vector<std::vector<Eigen::Matrix3d>> endPtCov2, Eigen::Matrix3d& R_best, Eigen::Vector3d& t_best);

};
