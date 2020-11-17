/*

Defines the functions of the Frame

Author  : Subramanian Krishnan
Date    : 30 Sep 2020

Changelog:
    subbu - 9/30 - Initial commit
*/

// Custom headers
#include "frame.h"

Frame::Frame(const cv::Mat& rgb_image, const cv::Mat& depth_image) : rgb_image(rgb_image), depth_image(depth_image) {}

FramePair::FramePair(const cv::Mat& rgb_image1, cv::Mat& depth_image1, cv::Mat& rgb_image2, cv::Mat& depth_image2) :    rgb_image1(rgb_image1), 
                                                                                                                        depth_image1(depth_image1),
                                                                                                                        rgb_image2(rgb_image2), 
                                                                                                                        depth_image2(depth_image2) {
    // Function returing lines in both images and matches between them contained in a structure element
    pstruct = image_process(rgb_image1, rgb_image2);

    img1_lines.resize(pstruct.linesInLeft.size(),4);
    img2_lines.resize(pstruct.linesInLeft.size(),4);

    for (int i = 0; i < pstruct.linesInLeft.size(); i++)
    {
        Eigen::Vector4d r1(pstruct.linesInLeft[i][0].startPointX, pstruct.linesInLeft[i][0].startPointY, pstruct.linesInLeft[i][0].endPointX, pstruct.linesInLeft[i][0].endPointY);
        img1_lines.row(i) = r1;
        Eigen::Vector4d r2(pstruct.linesInRight[i][0].startPointX, pstruct.linesInRight[i][0].startPointY, pstruct.linesInRight[i][0].endPointX, pstruct.linesInRight[i][0].endPointY);
        img2_lines.row(i) = r2;
    }
}
