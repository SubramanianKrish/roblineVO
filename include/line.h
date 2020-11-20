/*
This declares all methods dealing with lines

Changelog:
    awadhut - 10/?  - Wrote the line detector methods
    subbu   - 10/22 - initial commit during refactoring

*/
# pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>

// <maybe-todo> Is class the appropriate abstraction for the following functions?
class vo_line{

    public:
        cv::Mat edge_image;
        Eigen::MatrixXi line_endpoints;
        std::vector<std::vector<cv::Point2i>> sampled_lines_2d;
        Eigen::MatrixXd sampled_lines_eig;

        // Constructor for line class
        vo_line(const cv::Mat& rgb, const cv::Mat& depth);        
    
    private:
        // Canny edge detector wrapper 
        void GetEdgeImage(const cv::Mat& img, int lowThreshold, int highThreshold, int kernel_size, bool enable_blur);

        // Get the probabilistic hough lines
        // <TODO> Convert to LSD as we need matching later.
        Eigen::MatrixXi GetHoughLinesP(cv::Mat& edge_image, int thresh, int minLen, int maxGap);

        // Sampler for 2D lines
        Eigen::MatrixXd SampleIndices(const Eigen::MatrixXi& lines, const int& height, const int& width);
};
