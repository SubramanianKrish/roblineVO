/*
Contains the declaration of standard utilities used by roblineVO

Changelog:
    athube - 10/02 - Read/display image, drawline
    subbu  - 10/02 - drawcamera
*/

#pragma once

#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <vector> 
// Read in an image
cv::Mat ReadImage(std::string image_path);

// display a single image
void DisplayImage(cv::Mat img);

// draw line on image given the coordinates of endpoints
void DrawLine(cv::Mat img, int x1, int y1, int x2, int y2);

// draw camera using a rectangular pyramid
void DrawSingleCamera(const cv::Mat& camera_matrix, float w=1.0, float h_ratio=0.75, float z_ratio=0.6);

// Show two images side-by-side
void DisplayDualImage(const cv::Mat& image1, const cv::Mat& image2);

void DrawLines(std::vector<std::vector<cv::Point3d>> lines);

void DrawPoints(std::vector<cv::Point3d> points, std::vector<double> color, float point_size);

cv::Mat GetEdgeImage(cv::Mat& img, int lowThreshold, int highThreshold, int kernel_size, bool enable_blur);

cv::Mat DrawHoughLinesP(cv::Mat img, std::vector<cv::Vec4i> linesP);

std::vector<cv::Vec4i> GetHoughLinesP(cv::Mat& edge_image, int thresh, int minLen, int maxGap);

void drawCoordinates();