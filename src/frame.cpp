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
    pstruct = image_process(rgb_image1, rgb_image2);

}