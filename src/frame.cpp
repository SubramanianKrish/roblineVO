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
// Frame::Frame(const cv::Mat& rgb_image, const cv::Mat& depth_image) : rgb_image(rgb_image), depth_image(depth_image) {}

FramePair::FramePair(const cv::Mat& rgb_image1, cv::Mat& depth_image1, cv::Mat& rgb_image2, cv::Mat& depth_image2) :    rgb_image1(rgb_image1), 
                                                                                                                        depth_image1(depth_image1),
                                                                                                                        rgb_image2(rgb_image2), 
                                                                                                                        depth_image2(depth_image2) {
    // Function returing lines in both images and matches between them contained in a structure element
    pstruct = image_process(rgb_image1, rgb_image2);

    img1_lines.resize(int(pstruct.matches.size()/2),4);
    img2_lines.resize(int(pstruct.matches.size()/2),4);

    int lineIDLeft;
    int lineIDRight;
    for (unsigned int pair = 0; pair < pstruct.matches.size() / 2; pair++)
    {
        lineIDLeft = pstruct.matches[2 * pair];
        lineIDRight = pstruct.matches[2 * pair + 1];
        Eigen::Vector4i r1(int(pstruct.linesInLeft[lineIDLeft][0].startPointX), int(pstruct.linesInLeft[lineIDLeft][0].startPointY), 
                            int(pstruct.linesInLeft[lineIDLeft][0].endPointX), int(pstruct.linesInLeft[lineIDLeft][0].endPointY));
        Eigen::Vector4i r2(int(pstruct.linesInRight[lineIDRight][0].startPointX), int(pstruct.linesInRight[lineIDRight][0].startPointY), 
                            int(pstruct.linesInRight[lineIDRight][0].endPointX), int(pstruct.linesInRight[lineIDRight][0].endPointY));
        img1_lines.row(pair) = r1;
        img2_lines.row(pair) = r2;

        /*
        Following code allows to visualize individual matches between the two images
        */

        // cv::Point startPoint = cv::Point(int(pstruct.linesInLeft[lineIDLeft][0].startPointX), int(pstruct.linesInLeft[lineIDLeft][0].startPointY));
        // cv::Point endPoint = cv::Point(int(pstruct.linesInLeft[lineIDLeft][0].endPointX), int(pstruct.linesInLeft[lineIDLeft][0].endPointY));
        // cv::line(rgb_image1, startPoint, endPoint, CV_RGB(255,0,0), 1, cv::LINE_AA, 0);
        
        // startPoint = cv::Point(int(pstruct.linesInRight[lineIDRight][0].startPointX), int(pstruct.linesInRight[lineIDRight][0].startPointY));
        // endPoint = cv::Point(int(pstruct.linesInRight[lineIDRight][0].endPointX), int(pstruct.linesInRight[lineIDRight][0].endPointY));
        // cv::line(rgb_image2, startPoint, endPoint, CV_RGB(255,0,0), 1, cv::LINE_AA, 0);


        // utils::DisplayDualImage(rgb_image1, rgb_image2);
        // cv::waitKey(0);
    }

}
