#include <opencv2/opencv.hpp>
# include <Eigen/Dense>

#include "line.h"

cv::Mat vo_line::GetEdgeImage(cv::Mat& img, int lowThreshold = 50, int highThreshold = 150, int kernel_size = 3, bool enable_blur = false){
    cv::Mat smooth_image, gray_image, edge_image;
    if (enable_blur){
        cv::blur(img, smooth_image, cv::Size(5, 5), cv::Point(-1,-1) );
    }
    else{
        smooth_image = img;
    }
    cv::cvtColor(smooth_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Canny(gray_image, edge_image, lowThreshold, highThreshold, kernel_size);
    return edge_image;
}

std::vector<cv::Vec4i> vo_line::GetHoughLinesP(cv::Mat& edge_image, int thresh = 50, int minLen = 50, int maxGap = 10){
    std::vector<cv::Vec4i> linesP;
    cv::HoughLinesP(edge_image, linesP, 1, CV_PI/180, thresh, minLen, maxGap);
    return linesP;
}

cv::Mat vo_line::DrawHoughLinesP(cv::Mat img, std::vector<cv::Vec4i> linesP){
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    return img;
}

std::vector<std::vector<cv::Point2i>> vo_line::SampleIndices(const std::vector<cv::Vec4i>& lines, const int& ht, const int& wd){
    /*
    Input: lines vector expected to be of form nx4 with each entry being [x1,y1,x2,y2] 
           where x_i, y_i denote the end points of the lines
    
    Output: Sampled points of the form (2n)x(n_samples) vector.
    note: different lines have different n_samples based on line length
    */
   
   std::vector<std::vector<cv::Point2i>> sampled_lines;
   int max_samples = 100;

    for(auto& line: lines){
        Eigen::Vector2f p1(line[0], line[1]), p2(line[2], line[3]);
        float dist = (p1-p2).norm();
        float n_samples = std::min(max_samples, int(dist));

        // ref: https://stackoverflow.com/questions/28018147/emgucv-get-coordinates-of-pixels-in-a-line-between-two-points
        int x0 = line[0], y0 = line[1], x1 = line[2], y1 = line[3];
        int dx = std::abs(x1- x0), dy = std::abs(y1-y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        // For each line store the index points (need to be 'int')
        std::vector<cv::Point2i> line_points;

        if (n_samples < 100){
            // Bresenham's algorithm
            while(true){
                line_points.push_back(cv::Point2i(x0,y0));

                if(x0 == x1 && y0 == y1) break;
                int e2 = 2*err;
                if(e2 > -dy){
                    err = err - dy;
                    x0 = x0 + sx;
                }
                if(e2 < dx){
                    err = err + dx;
                    y0 = y0 + sy;
                }
            }

        }
        else{
         for (int i=1; i<=max_samples; ++i){
            line_points.push_back(cv::Point2i(x0 + (x1 - x0)*(i-1)/(max_samples-1), y0 + (y1 - y0)*(i-1)/(max_samples-1)));
         }
        }
        // Add the x and y indices for returning later
        sampled_lines.push_back(line_points);
    }
    return sampled_lines;
}


std::vector<cv::Point3d> vo_line::Reproject(const cv::Mat& rgb_image, const cv::Mat& depth_image, const std::vector<std::vector<cv::Point2i>> sampled_lines_2d){
    // Ugh - convert to cv mat for matrix operation
    for(auto& line: sampled_lines_2d){
        std::vector<cv::Point3d> line_3d;
        for (auto& point: line){
            continue;
        }
    }
}
