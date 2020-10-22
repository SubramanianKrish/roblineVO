/*
Contains the declaration of standard utilities used by roblineVO

Changelog:
    athube - 10/02 - Read/display image, drawline
    subbu  - 10/02 - drawcamera
    ...
    subbu - 10/20 - no progress -_-
    subbu - 10/22 - refactor
*/

#pragma once

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace utils{
    // Read in an image
    cv::Mat ReadImage(const std::string& image_path);
    
    // Display a single image
    void DisplayImage(const cv::Mat& img);

    // Show two images side-by-side
    void DisplayDualImage(const cv::Mat& image1, const cv::Mat& image2);
    
    // Draw line on image given the coordinates of endpoints
    void DrawLine2D(const cv::Mat& img, const int& x1, const int& y1, const int& x2, const int& y2);
    
    // Draw lines in 3D between two points (x1,y1,z1) and (x2,y2,z2)
    // <TODO> May need to use cv::Mat to represent the lines [nx3] and n->even as each 
    // pair would represent a line.
    void DrawLines3D(const std::vector<std::vector<cv::Point3d>>& lines);
    
    // Draw camera using a rectangular pyramid
    void DrawSingleCamera(const cv::Mat& camera_pose, const float& w=1.0, const float& h_ratio=0.75, const float& z_ratio=0.6);

    // Draw points in 3D
    // <TODO> May need to work with cv::mat [nx3] as the input along with their color information
    // <TODO> Add default color information if no color is passed
    void DrawPoints3D(const std::vector<cv::Point3d>& points, const std::vector<double>& color, const float& point_size);

    // Draw points sampled on lines in 2D
    // <TODO> May need to work with cv::Mat [nx2] points as the input
    // <TODO> May need to rename as drawpoints2D
    void DrawSampledLines2D(const cv::Mat& img, const std::vector<std::vector<cv::Point2i>>& sampled_lines);

    // This draws the world coordinate frame at the world origin
    void DrawCoordinates();
}
