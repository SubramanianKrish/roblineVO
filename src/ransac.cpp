#include <iostream>
#include <Eigen/Geometry>

#include "ransac.h"

using namespace std;

double Ransac::mahalanobis_distance(const point3d& X, const covariance& sigma_x, const point3d& A, const point3d& B){
    Eigen::SelfAdjointEigenSolver<covariance> eigensolver(sigma_x);
    
    if (eigensolver.info() != Eigen::Success){
        cout << "Could not perform eigen decomposition on 3D point's covariance matrix" << endl;
        cout << sigma_x << endl;
        abort();
    }

    Eigen::Matrix3d U = eigensolver.eigenvectors();
    Eigen::Vector3d D = eigensolver.eigenvalues();

    // Apply affine transform to points
    Eigen::Vector3d A_affine = ((D.array().inverse()).sqrt()).matrix().asDiagonal() * U.transpose() * (A - X);
    Eigen::Vector3d B_affine = ((D.array().inverse()).sqrt()).matrix().asDiagonal() * U.transpose() * (B - X);

    double distance = A_affine.cross(B_affine).norm()/(A_affine - B_affine).norm();

    return distance;
}
