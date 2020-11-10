/*

We define a data frame as a time synchronised RGB and Depth
image.

Author  : Subramanian Krishnan
Date    : 30 Sep 2020

Changelog:
    subbu - 9/30 - Initial commit
*/

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "LineMatchingAlgorithm.hh"
typedef struct PairData pairStruct;

class Frame{
    public:
        cv::Mat rgb_image;
        cv::Mat depth_image;
        // Constructor for a frame with just RGB image
        Frame(const cv::Mat& rgb_image, const cv::Mat& depth_image);
};

class FramePair{
    public:
        cv::Mat rgb_image1;
        cv::Mat depth_image1;
        cv::Mat rgb_image2;
        cv::Mat depth_image2;
        pairStruct pstruct;
        FramePair(const cv::Mat& rgb_image1, cv::Mat& depth_image1, cv::Mat& rgb_image2, cv::Mat& depth_image2);
};