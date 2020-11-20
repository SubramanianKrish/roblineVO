/*

Defines the functions of the Frame

Author  : Subramanian Krishnan
Date    : 30 Sep 2020

Changelog:
    subbu - 9/30  - Initial commit
    subbu - 10/27 - update members+methods
*/

#include <iostream>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

// Custom headers
#include "frame.h"
#include "utils.h"

using namespace Eigen;

Eigen::MatrixXd Frame::Reproject(const cv::Mat& depth_image, const Eigen::MatrixXd& sampled_lines){
    Eigen::MatrixXd points_3d(3, sampled_lines.rows());

    // This is slow. Need to find a better way :'<
    for(int i=0; i<sampled_lines.rows(); ++i){
        points_3d(0,i) = (sampled_lines(i,0) - K(0,2))*depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1))/K(0,0);
        points_3d(1,i) = (sampled_lines(i,1) - K(1,2))*depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1))/K(1,1);
        points_3d(2,i) = depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1));
    }

    return points_3d;
}

Frame::Frame(const cv::Mat& rgb_image, const cv::Mat& depth_image)
       : rgb_image(rgb_image), depth_image(depth_image) {
           
           // Make the intrinsic matrix [Got from dataset information]
           K = (Eigen::Matrix<double, 3, 3>() << 517.306408, 0.000000, 318.643040, 
                                                 0.000000, 516.469215, 255.313989,
                                                 0.000000, 0.000000, 1.000000).finished();

           // populate distortion
           dist = {0.262383, -0.953104, -0.005358, 0.002628, 1.163314};

           // populate image height and width
           im_wd = rgb_image.rows;
           im_ht = rgb_image.cols;

           // Populate the line class
           lines = std::make_shared<vo_line>(rgb_image, depth_image);

           // Reprojection function to get 3D points
           points_3d = Reproject(depth_image, lines->sampled_lines_eig);
       }
