/*

We define a data frame as a time synchronised RGB and Depth
image.

Author  : Subramanian Krishnan
Date    : 30 Sep 2020

Changelog:
    subbu - 9/30 - Initial commit
*/

#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "LineMatchingAlgorithm.hh"
typedef struct PairData pairStruct;

#include <memory>

#include "line.h"

class Frame{
    public:
        cv::Mat rgb_image, depth_image;
        
        std::shared_ptr<vo_line> lines;

        Eigen::MatrixXd points_3d;

        // reprojection function
        Eigen::MatrixXd Reproject(const cv::Mat& depth, const Eigen::MatrixXd& sampled_lines);
        // Constructor for a frame
        Frame(const cv::Mat& rgb_image, const cv::Mat& depth_image);


    private:
        int im_wd, im_ht; // height and width of image
        std::vector<float> dist; // distortion parameters
        Eigen::Matrix<double, 3, 3> K;  // Camera intrinsics
};

class FramePair{
    /*
    Class containing all required information about a pair of consecutive images
    */
    public:
        cv::Mat rgb_image1;
        cv::Mat depth_image1;
        cv::Mat rgb_image2;
        cv::Mat depth_image2;

        Eigen::Matrix<int, Eigen::Dynamic, 4> img1_lines;
        Eigen::Matrix<int, Eigen::Dynamic, 4> img2_lines;
        
        // Structure FramePair defined in ../LBD_and_LineMatching/LineStructure.hh
        pairStruct pstruct;
        FramePair(const cv::Mat& rgb_image1, cv::Mat& depth_image1, cv::Mat& rgb_image2, cv::Mat& depth_image2);
};