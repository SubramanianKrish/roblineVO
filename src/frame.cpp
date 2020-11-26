/*

Defines the functions of the Frame

Author  : Subramanian Krishnan
Date    : 30 Sep 2020

Changelog:
    subbu - 9/30  - Initial commit
    subbu - 10/27 - update members+methods
*/

#include <iostream>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

// Custom headers
#include "frame.h"
#include "utils.h"

using namespace Eigen;

Eigen::MatrixXd Frame::Reproject(const cv::Mat& depth_image, const Eigen::MatrixXd& sampled_lines){
    Eigen::MatrixXd points_3d(3, sampled_lines.rows());
    // This is slow. Need to find a better way :'<
    for(int i=0; i<sampled_lines.rows(); ++i){
        points_3d(0,i) = (sampled_lines(i,0) - K(0,2))*depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1))/K(0,0);
        points_3d(1,i) = (sampled_lines(i,1) - K(1,2))*depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1))/K(1,1);
        points_3d(2,i) = depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1));
    }

    return points_3d;
}

Frame::Frame(const cv::Mat& rgb_image, const cv::Mat& depth_image)
       : rgb_image(rgb_image), depth_image(depth_image) {
           
           // Make the intrinsic matrix [Got from dataset information]
           K = (Eigen::Matrix<double, 3, 3>() << 517.306408, 0.000000, 318.643040, 
                                                 0.000000, 516.469215, 255.313989,
                                                 0.000000, 0.000000, 1.000000).finished();

           // populate distortion
           dist = {0.262383, -0.953104, -0.005358, 0.002628, 1.163314};

           // populate image height and width
           im_wd = rgb_image.rows;
           im_ht = rgb_image.cols;

           // Populate the line class
           lines = std::make_shared<vo_line>(rgb_image, depth_image);

           // Reprojection function to get 3D points
           points_3d = Reproject(depth_image, lines->sampled_lines_eig);
       }

Eigen::MatrixXd FramePair::SampleIndices(const Eigen::MatrixXi& lines, const int& ht, const int& wd, std::vector<std::vector<cv::Point2i>>& sampled_lines_2d ){
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

Eigen::MatrixXd FramePair::Reproject(const cv::Mat& depth_image, const Eigen::MatrixXd& sampled_lines){
    // Eigen::MatrixXd points_3d(3, sampled_lines.rows());
    Eigen::MatrixXd points_3d(3, 640*480);
    // This is slow. Need to find a better way :'<
    // for(int i=0; i<sampled_lines.rows(); ++i){
    //     points_3d(0,i) = (sampled_lines(i,0) - K(0,2))*depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1))/(K(0,0)*5000);
    //     points_3d(1,i) = (sampled_lines(i,1) - K(1,2))*depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1))/(K(1,1)*5000);
    //     points_3d(2,i) = depth_image.at<float>(sampled_lines(i,0), sampled_lines(i,1))/5000;
    // }

    for(int i=0; i < 480; ++i){
        for(int j=0; j<640; ++j){
            points_3d(0,i*640+j) = (j - K(0,2))*depth_image.at<uint16_t>(i,j)/(K(0,0)*5000.0);
            points_3d(1,i*640+j) = (i - K(1,2))*depth_image.at<uint16_t>(i,j)/(K(1,1)*5000.0);
            points_3d(2,i*640+j) = depth_image.at<uint16_t>(i,j)/5000.0;
        }
    }

    return points_3d;
}

FramePair::FramePair(const cv::Mat& rgb_image1, cv::Mat& depth_image1, cv::Mat& rgb_image2, cv::Mat& depth_image2) :    rgb_image1(rgb_image1), 
                                                                                                                        depth_image1(depth_image1),
                                                                                                                        rgb_image2(rgb_image2), 
                                                                                                                        depth_image2(depth_image2) {
    // Make the intrinsic matrix [Got from dataset information]
    K = (Eigen::Matrix<double, 3, 3>() << 517.306408, 0.000000, 318.643040, 
                                            0.000000, 516.469215, 255.313989,
                                            0.000000, 0.000000, 1.000000).finished();

    // populate distortion
    dist = {0.262383, -0.953104, -0.005358, 0.002628, 1.163314};

    // Function returing lines in both images and matches between them contained in a structure element
    pstruct = image_process(rgb_image1, rgb_image2);

    img1_lines.resize(int(pstruct.matches.size()/2),4);
    img2_lines.resize(int(pstruct.matches.size()/2),4);

    int lineIDLeft;
    int lineIDRight;
    for (unsigned int pair = 0; pair < pstruct.matches.size() / 2; pair++)
    {
        lineIDLeft = pstruct.matches[2 * pair];
        lineIDRight = pstruct.matches[2 * pair + 1];
        Eigen::Vector4i r1(int(pstruct.linesInLeft[lineIDLeft][0].startPointX), int(pstruct.linesInLeft[lineIDLeft][0].startPointY), 
                            int(pstruct.linesInLeft[lineIDLeft][0].endPointX), int(pstruct.linesInLeft[lineIDLeft][0].endPointY));
        Eigen::Vector4i r2(int(pstruct.linesInRight[lineIDRight][0].startPointX), int(pstruct.linesInRight[lineIDRight][0].startPointY), 
                            int(pstruct.linesInRight[lineIDRight][0].endPointX), int(pstruct.linesInRight[lineIDRight][0].endPointY));
        img1_lines.row(pair) = r1;
        img2_lines.row(pair) = r2;

        /*
        // ---------Following code allows to visualize individual matches between the two images-------------
        cv::Point startPoint = cv::Point(int(pstruct.linesInLeft[lineIDLeft][0].startPointX), int(pstruct.linesInLeft[lineIDLeft][0].startPointY));
        cv::Point endPoint = cv::Point(int(pstruct.linesInLeft[lineIDLeft][0].endPointX), int(pstruct.linesInLeft[lineIDLeft][0].endPointY));
        cv::line(rgb_image1, startPoint, endPoint, CV_RGB(255,0,0), 1, cv::LINE_AA, 0);
        startPoint = cv::Point(int(pstruct.linesInRight[lineIDRight][0].startPointX), int(pstruct.linesInRight[lineIDRight][0].startPointY));
        endPoint = cv::Point(int(pstruct.linesInRight[lineIDRight][0].endPointX), int(pstruct.linesInRight[lineIDRight][0].endPointY));
        cv::line(rgb_image2, startPoint, endPoint, CV_RGB(255,0,0), 1, cv::LINE_AA, 0);
        utils::DisplayDualImage(rgb_image1, rgb_image2);
        cv::waitKey(0);
        */

    }

    // sample lines in image [copying result to member may be slow <to-do> populate inside function]
    sampled_lines_eig_left = SampleIndices(img1_lines, rgb_image1.rows, rgb_image1.cols, sampled_lines_2d_left);
    sampled_lines_eig_right = SampleIndices(img2_lines, rgb_image2.rows, rgb_image2.cols, sampled_lines_2d_right);

    // Reproject left and right image points to 3D
    points_3d_im1 = Reproject(depth_image1, sampled_lines_eig_left);
    points_3d_im2 = Reproject(depth_image2, sampled_lines_eig_right);

}
