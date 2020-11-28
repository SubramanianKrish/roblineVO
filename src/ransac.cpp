#include <iostream>
#include <random>
#include <Eigen/Geometry>

#include "ransac.h"

using namespace std;

Ransac::Ransac(int n_iterations, double threshold): n_iterations(n_iterations),
                                                    threshold(threshold)
{    // Seed the generator
    generator.seed(rd());
};

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

Eigen::Matrix3Xd Ransac::removeOutlierPoints(const Eigen::Matrix3Xd& linepoints, const std::vector<Eigen::Matrix3d>& cov){
    Eigen::Matrix3Xd best_inliers;
    int best_num_inliers = -1;
    float distance;

    // Setup the range from which we choose random numbers
    std::uniform_int_distribution<unsigned> distribution(0,linepoints.cols()-1); // end inclusive

    for(int iter=0; iter < n_iterations; ++iter){
        int inliers = 0;
        int i = distribution(generator), j = distribution(generator);
        Eigen::Matrix3Xd support_points;
        Eigen::Vector3d A = linepoints.col(i), B = linepoints.col(j);

        // calculate support for chosen endpoint indices
        for(int k=0; k < linepoints.cols(); ++k){
            if(k==i or k==j) continue;
            distance = mahalanobis_distance(linepoints.col(k), cov[k], A, B);
            cout << distance << endl;
            if (distance < threshold) support_points.col(inliers++) = linepoints.col(k);
        }

        if(inliers > best_num_inliers){
            best_num_inliers = inliers;
            best_inliers = support_points.leftCols(inliers);
        }
    }
    
    return best_inliers;
}
