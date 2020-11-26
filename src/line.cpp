#include <opencv2/opencv.hpp>
# include <Eigen/Dense>

#include "line.h"

void vo_line::GetEdgeImage(const cv::Mat& img, int lowThreshold = 50, int highThreshold = 150, int kernel_size = 3, bool enable_blur = false){
    cv::Mat smooth_image, gray_image;
    if (enable_blur){
        cv::blur(img, smooth_image, cv::Size(5, 5), cv::Point(-1,-1) );
    }
    else{
        smooth_image = img;
    }
    cv::cvtColor(smooth_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Canny(gray_image, edge_image, lowThreshold, highThreshold, kernel_size);
}

Eigen::MatrixXi vo_line::GetHoughLinesP(cv::Mat& edge_image, int thresh = 50, int minLen = 50, int maxGap = 10){
    /*
    Input   edge_image: Binary image (output of canny edge detector)
            thresh, minLen, maxGap: Line extraction parameters (see OpenCV - houghlinesP)

    Output  m:  Eigen matrix containing endpoints of lines found in the image (Nx4)
    */

    std::vector<cv::Vec4i> linesP;
    cv::HoughLinesP(edge_image, linesP, 1, CV_PI/180, thresh, minLen, maxGap);
    Eigen::MatrixXi mat(linesP.size(),4);
    for (int i = 0; i < linesP.size(); i++)
    {
        Eigen::Vector4i r(linesP[i][0], linesP[i][1], linesP[i][2], linesP[i][3]);
        mat.row(i) = r;
    }
    return mat;
}

Eigen::MatrixXd vo_line::SampleIndices(const Eigen::MatrixXi& lines, const int& ht, const int& wd){
    /*
    Input: lines matrix expected to be of form nx4 with each entry being [x1,y1,x2,y2] 
           where x_i, y_i denote the end points of the lines
    
    Output: Sampled points of the form (2n)x(n_samples) vector.
    note: different lines have different n_samples based on line length
    */
   
    int max_samples = 100;
    u_int num_points = 0;

    // Iterating through row elements is possible in eigen 3.4
    for(int i = 0; i < lines.rows(); i++ ){        
        Eigen::Vector2i p1(lines(i,0), lines(i,1)), p2(lines(i,2), lines(i,3));
        float dist = (p1-p2).norm();
        float n_samples = std::min(max_samples, int(dist));

        // ref: https://stackoverflow.com/questions/28018147/emgucv-get-coordinates-of-pixels-in-a-line-between-two-points
        int x0 = lines(i,0), y0 = lines(i,1), x1 = lines(i,2), y1 = lines(i,3);
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
                num_points++;

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
            num_points++;
         }
        }
        // Add the x and y indices to class variable
        sampled_lines_2d.push_back(line_points);
    }

    //convert to an eigen matrix of points [this maybe stupidly slow]
    Eigen::MatrixXd sampled_indices(num_points,2);
    for (int i=0, eig_index=0; i<lines.rows(); ++i){
        size_t num_points_in_line = sampled_lines_2d[i].size();
        for(int j=0; j<num_points_in_line; ++j){
            Eigen::Vector2d index_2d(sampled_lines_2d[i][j].x, sampled_lines_2d[i][j].y);
            sampled_indices.row(eig_index++) = index_2d;
        }
    }

    return sampled_indices;

}

vo_line::vo_line(const cv::Mat& rgb_image, const cv::Mat& depth_image){
    // populate edge image in line class
    vo_line::GetEdgeImage(rgb_image);

    // populate the detected lines in houghP [edge_image avbl in class]
    line_endpoints = vo_line::GetHoughLinesP(edge_image);

    // sample lines in image [copying result to member may be slow <to-do> populate inside function]
    sampled_lines_eig = vo_line::SampleIndices(line_endpoints, rgb_image.rows, rgb_image.cols);
}
