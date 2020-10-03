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

class Frame{
    public:
        cv::Mat rgb_image;

        // Constructor for a frame with just RGB image
        Frame(const cv::Mat& rgb_image);
};
