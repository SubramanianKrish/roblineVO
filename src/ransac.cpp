#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <memory>

#include "ransac.h"
#include "optim.h"
using namespace std;

Ransac::Ransac(int n_iterations, double threshold): n_iterations(n_iterations),
                                                    threshold(threshold)
{    // Seed the generator
    generator.seed(rd());
};

double Ransac::mahalanobis_distance(const point3d& X, const point3d& A, const point3d& B,
                                    const Eigen::Vector3d& D, const Eigen::Matrix3d& U)
{
    // Apply affine transform to points
    Eigen::Vector3d A_affine = ((D.array().inverse()).sqrt()).matrix().asDiagonal() * U.transpose() * (A - X);
    Eigen::Vector3d B_affine = ((D.array().inverse()).sqrt()).matrix().asDiagonal() * U.transpose() * (B - X);

    double distance = A_affine.cross(B_affine).norm()/(A_affine - B_affine).norm();

    return distance;
}

Eigen::Matrix3Xd Ransac::removeOutlierPoints(const Eigen::Matrix3Xd& linepoints, const std::vector<Eigen::Vector3d>& eig_val,
                                             const std::vector<Eigen::Matrix3d>& eig_vector, std::vector<Eigen::Matrix3d>& updated_covariance,
                                             std::vector<Eigen::Matrix3d>& updated_inv_root_covariance, const std::vector<Eigen::Matrix3d>& cov_G,
                                             const std::vector<Eigen::Matrix3d>& inv_cov_G)
{
    Eigen::Matrix3Xd best_inliers;
    int best_num_inliers = -1;
    float distance;

    // solace in not copying this over and over
    std::shared_ptr<vector<int>> ptr_to_best_inlier_indices = NULL;
    
    // Setup the range from which we choose random numbers
    std::uniform_int_distribution<unsigned> distribution(0,linepoints.cols()-1); // end inclusive
    
    for(int iter=0; iter < n_iterations; ++iter){
        int inliers = 0;
        auto ptr_to_inlier_indices = std::make_shared<std::vector<int>>();

        // re-generate i and j randomly until they are not equal
        int i = distribution(generator), j = distribution(generator);
        while(i == j){
            i = distribution(generator), j = distribution(generator);
        }
        
        Eigen::Matrix3Xd support_points(3, linepoints.cols());
        Eigen::Vector3d A = linepoints.col(i), B = linepoints.col(j);

        // calculate support for chosen endpoint indices
        for(int k=0; k < linepoints.cols(); ++k){
            distance = mahalanobis_distance(linepoints.col(k), A, B, eig_val[k], eig_vector[k]);
            // cout << distance << endl;
            if (distance < threshold){
                support_points.col(inliers++) = linepoints.col(k);
                ptr_to_inlier_indices->push_back(k);
            }

        }

        if(inliers > best_num_inliers){
            best_num_inliers = inliers;
            best_inliers = support_points.leftCols(inliers);
            ptr_to_best_inlier_indices = ptr_to_inlier_indices;
        }
    }

    // Take the indices of new covariances and populate updated covariances
    // Very slow - <To-do> refactor to deal with indices/pointers instead of copying over data
    for(auto i: *ptr_to_best_inlier_indices){
        updated_covariance.push_back(cov_G[i]);
        updated_inv_root_covariance.push_back(inv_cov_G[i]);
    }
    
    return best_inliers;
}

std::vector<int> Ransac::ransac3D(const std::vector<points3d>& optLines1, const std::vector<points3d>& optLines2, const std::vector<std::vector<Eigen::Matrix3d>>& endPtCov1, 
                const std::vector<std::vector<Eigen::Matrix3d>> endPtCov2, Eigen::Matrix3d& R_best, Eigen::Vector3d& t_best)
{
    //  Num of iterations of ransac
    // int num_iter = 100;
    float maha_dist;
    int best_num_inliers = 0;
    // double threshold = 5.0;
    std::uniform_int_distribution<unsigned> distribution(0,optLines1.size() - 1);
    // std::default_random_engine generator;

    std::shared_ptr<vector<int>> ptr_to_best_inlier_indices = NULL;
    for (int k = 0; k < 200; k++)
    {
        int inliers = 0;
        auto ptr_to_inlier_indices = std::make_shared<std::vector<int>>();
        // Random integer generation
        int i = distribution(generator), j = distribution(generator);
        while(i == j){
            i = distribution(generator), j = distribution(generator);
        }

        // Getting random lines from the first image
        optim::LineData l1(optLines1[i].leftCols(1), optLines1[i].rightCols(1));
        optim::LineData l2(optLines1[j].leftCols(1), optLines1[j].rightCols(1));

        // checking if the lines are parallel
        // don't hardcode the threshold (threshold = cos(10))

        // Getting corresponding lines from the second image
        optim::LineData l1_prime(optLines2[i].leftCols(1), optLines2[i].rightCols(1));
        optim::LineData l2_prime(optLines2[j].leftCols(1), optLines2[j].rightCols(1));

        if (abs((l1_prime.u.dot(l2_prime.u))) > 0.984) 
            continue;

        std::vector<optim::LineData> im1_lines = {l1,l2}, im2_lines = {l1_prime, l2_prime};

        Eigen::Matrix3d Rot = optim::ComputeRotationMatrix(l1, l1_prime, l2, l2_prime);
        
        Eigen::Vector3d T = optim::ComputeTranslation(im1_lines, im2_lines, Rot);

        // std::cout << "Translation is = \n " << T << std::endl;

        for (int size = 0; size < optLines1.size(); size++)
        {
            // if (!((size == i) || (size == j)))
            //     continue;
            std::cout << "Indices " << i << " " << j << " " << size << "\n";
            optim::LineData l(optLines1[size].leftCols(1), optLines1[size].rightCols(1));
            optim::LineData l_prime(optLines2[size].leftCols(1), optLines2[size].rightCols(1));

            // double error = optim::computeRtError(Rot, T, l.A, l.B, endPtCov1[size][0], endPtCov1[size][1], 
            //                                     l_prime.A, l_prime.B, endPtCov2[size][0], endPtCov2[size][1]);


            double error = optim::computeRtError(Rot, T, l.A, l.B, l_prime.A, l_prime.B);
        

            if (error < 0.05)
            {
                std::cout << "Error is = " << error << std::endl;
                // std::cout << "Rotation = \n" << Rot << "\n";
                // std::cout << "Translation = \n" << T << "\n";
                inliers++;
                ptr_to_inlier_indices->push_back(size);
            }
        }
        if(inliers > best_num_inliers){
            R_best = Rot;
            t_best = T;
            best_num_inliers = inliers;
            ptr_to_best_inlier_indices = ptr_to_inlier_indices;
        }
    }
    int inp;
    std::cout << "Number of possible inliers " << optLines1.size() << std::endl;
    std::cout << "Number of best Inliers " << best_num_inliers << std::endl;
    std::cout << "Best R " << R_best << std::endl;
    std::cout << "Best T " << t_best << std::endl;
    std::cin >> inp;
    std::vector<int> result;
    for(auto i : *ptr_to_best_inlier_indices)
        result.push_back(i);

    std::cout << "Check 8" << std::endl;
    return result;
    
}