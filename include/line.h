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

    // Could have all the bookkeeping stuff and line representations as public members

    public:

        // Canny edge detector wrapper 
        cv::Mat GetEdgeImage(cv::Mat& img, int lowThreshold, int highThreshold, int kernel_size, bool enable_blur);

        // Get the probabilistic hough lines
        // <TODO> Convert to LSD as we need matching later
        Eigen::MatrixXi GetHoughLinesP(cv::Mat& edge_image, int thresh, int minLen, int maxGap);

        // Sampler for 2D lines
        std::vector<std::vector<cv::Point2i>> SampleIndices(const Eigen::MatrixXi& lines, const int& height, const int& width);
        
        // Reprojection function to get 3D points
        // <TODO> Should use cv::Mat [nx2] or [2xn] as input
        // <TODO> Should return cv::Mat [nx3] as output
        std::vector<cv::Point3d> Reproject(const cv::Mat& rgb_image, const cv::Mat& depth_image, const std::vector<std::vector<cv::Point2i>> points2d);

        // Draw lines on 2D image
        // <TODO?> Why is this not wrapping over drawlines 2D? more so why exist?
        cv::Mat DrawHoughLinesP(cv::Mat img, Eigen::MatrixXi linesP);
};
