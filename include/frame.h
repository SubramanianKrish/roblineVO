/*

We define a data frame as a time synchronised RGB and Depth
image.

Author  : Subramanian Krishnan
Date    : 30 Sep 2020

Changelog:
    subbu - 9/30 - Initial commit
    ...
    Subbu - 11/27 - refactor for lines/3D points
*/

#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <memory>

#include "ransac.h"
#include "line.h"
#include "LineMatchingAlgorithm.hh"

typedef struct PairData pairStruct;
// <to-do> push the points2d/3d into robline namespace. There's a cv::point2d too
typedef Eigen::Matrix<int,    2, Eigen::Dynamic> points2d;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> points3d;

class FramePair{
    /*
    Class containing all required information about a pair of consecutive images
    */
    public:
        // Previous frame
        cv::Mat rgb_image1;
        cv::Mat depth_image1;

        // Current Frame
        cv::Mat rgb_image2;
        cv::Mat depth_image2;

        // Lines in eigen form of both frames
        Eigen::Matrix<int, Eigen::Dynamic, 4> img1_lines;
        Eigen::Matrix<int, Eigen::Dynamic, 4> img2_lines;
        
        // Each element of the vector is a set of line samples in 2D
        std::vector<points2d> sampled_lines_2d_im1, sampled_lines_2d_im2;

        // 3d points in both images. Each element is set of line samples in 3D
        std::vector<points3d> points_3d_im1, points_3d_im2;
        // Ransac refined 3d points
        std::vector<points3d> rsac_points_3d_im1, rsac_points_3d_im2;

        // Structure FramePair defined in ../LBD_and_LineMatching/LineStructure.hh
        // Used to store line matches returned by the LBD matcher
        pairStruct pstruct;
        
        // Camera parameters
        std::vector<float> dist; // distortion parameters
        Eigen::Matrix<double, 3, 3> K;  // Camera intrinsics
        float im_ht, im_wd;

        // Covariance of 3D points in lines
        std::vector<std::vector<Eigen::Matrix3d>> cov_G_im1;
        std::vector<std::vector<Eigen::Matrix3d>> cov_G_im2;
        
        // EigenaValues of cov matrices [For ransac]
        std::vector<std::vector<Eigen::Vector3d>> cov_eig_values_im1;
        std::vector<std::vector<Eigen::Vector3d>> cov_eig_values_im2;

        // EigenVectors of cov matrices [For ransac]
        std::vector<std::vector<Eigen::Matrix3d>> cov_eig_vectors_im1;
        std::vector<std::vector<Eigen::Matrix3d>> cov_eig_vectors_im2;

        // Ransac object for culling outliers
        Ransac *pointRefine;

        // Line Sampler in 2D
        void SampleIndices(const Eigen::MatrixXi& lines, std::vector<points2d>& sampled_lines_2d);

        // Reprojection function
        void Reproject(const cv::Mat& depth, const std::vector<points2d>& sampled_lines,
                       std::vector<points3d>& reprojected_points, std::vector<std::vector<Eigen::Matrix3d>>& cov_G,
                       std::vector<std::vector<Eigen::Vector3d>>& frame_eig_values, std::vector<std::vector<Eigen::Matrix3d>>& frame_eig_vectors);

        // Covariance propogator
        Eigen::Matrix3d Cov3D(double u, double v, double depth);

        // ctor which populates members inside
        FramePair(const cv::Mat& rgb_image1, cv::Mat& depth_image1, cv::Mat& rgb_image2, cv::Mat& depth_image2);

};
