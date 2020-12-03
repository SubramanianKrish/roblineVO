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

void FramePair::SampleIndices(const Eigen::MatrixXi& lines, std::vector<points2d>& sampled_lines_2d){
    /*
    Input: lines matrix expected to be of form nx4 with each entry being [x1,y1,x2,y2] 
           where x_i, y_i denote the end points of the lines
    
    Output: Sampled points of the form (2n)x(n_samples) vector.
    note: different lines have different n_samples based on line length
    */
   
    const int max_samples = 100;

    // Iterating through row elements is possible in eigen 3.4
    for(int i = 0; i < lines.rows(); i++ ){        
        Eigen::Vector2i p1(lines(i,0), lines(i,1)), p2(lines(i,2), lines(i,3));
        float dist = (p1-p2).norm();
        float n_samples = std::min(max_samples, int(dist));
        u_int point_index = 0;

        // ref: https://stackoverflow.com/questions/28018147/emgucv-get-coordinates-of-pixels-in-a-line-between-two-points
        int x0 = lines(i,0), y0 = lines(i,1), x1 = lines(i,2), y1 = lines(i,3);
        int dx = std::abs(x1- x0), dy = std::abs(y1-y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        // For each line store the index points (need to be 'int')
        Eigen::Matrix<int, 2, max_samples> line_points;

        if (n_samples < max_samples){
            // Bresenham's algorithm
            while(true){
                line_points(0, point_index) = x0;
                line_points(1, point_index) = y0;
                point_index++;

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
            line_points(0, point_index) = x0 + (x1 - x0)*(i-1)/(max_samples-1);
            line_points(1, point_index) = y0 + (y1 - y0)*(i-1)/(max_samples-1);
            point_index++;
         }
        }
        // Add the x and y indices to class variable
        sampled_lines_2d.push_back(line_points.leftCols(point_index));
    }
}

Eigen::Matrix3d FramePair::Cov3D(double u, double v, double depth){
    // depth noise co-efficients for the kinect [ref: Lu, Song 2015]
    const double c1 = 0.00273, c2 = 0.00074, c3 = -0.00058;
    // Our estiamte of how accurate the 2D point sampled on a line is
    const double sigma_g = 3;
    double sigma_d = c1*depth*depth + c2*depth + c3;

    Eigen::Matrix3d cov2d = (Eigen::Matrix<double,3,3>()<< 
                             sigma_g*sigma_g, 0,               0,
                             0,               sigma_g*sigma_g, 0,
                             0,               0,               sigma_d*sigma_d).finished();

    Eigen::Matrix3d J = (Eigen::Matrix<double,3,3>()<< 
                         depth/K(0,0), 0,            u - K(0,2)/depth,
                         0,            depth/K(1,1), v - K(1,2)/depth,
                         0,            0,            1).finished();

    Eigen::Matrix3d cov3d = J*cov2d*(J.transpose());

    return cov3d;
}

void FramePair::Reproject(const cv::Mat& depth_image, const std::vector<points2d>& sampled_lines, std::vector<points3d>& s_lines_3d,
                          std::vector<std::vector<Eigen::Matrix3d>>& cov_G, std::vector<std::vector<Eigen::Vector3d>>& frame_eigen_values,
                          std::vector<std::vector<Eigen::Matrix3d>>& frame_eigen_vectors){
    for(auto& current_line: sampled_lines){
        Eigen::MatrixXd P(3, current_line.cols());
        
        std::vector<Eigen::Matrix3d> line_covariance;
        std::vector<Eigen::Matrix3d> eigen_vectors;
        std::vector<Eigen::Vector3d> eigen_values;

        // This is slow. Need to find a better way: can try K inverse
        int index = 0;
        for(int i=0; i<current_line.cols(); ++i){
            
            // rerpoject to 3D using depth value
            int u = current_line(0,i), v = current_line(1,i);
            float depth = depth_image.at<uint16_t>(v, u)/5000.0;
            if(depth == 0) continue;
            P(0,index) = (u - K(0,2))*depth/K(0,0);
            P(1,index) = (v - K(1,2))*depth/K(1,1);
            P(2,index) = depth;
            index++;
            
            // Propogate 2D covariance to 3D
            Eigen::Matrix3d covariance_3d = Cov3D(u, v, depth);

            // Generate Eigen values of cov for ransac later
            Eigen::SelfAdjointEigenSolver<covariance> eigensolver(covariance_3d);
            if (eigensolver.info() != Eigen::Success){
                cout << "Could not perform eigen decomposition on 3D point's covariance matrix" << endl;
                cout << covariance_3d << endl;
                abort();
            }
            
            Eigen::Matrix3d U = eigensolver.eigenvectors();
            Eigen::Vector3d D = eigensolver.eigenvalues();

            line_covariance.push_back(covariance_3d);
            eigen_values.push_back(D);
            eigen_vectors.push_back(U);
        }
        
        // Need at least 5 3D points to even consider it as a line <Not in paper>
        if(index >= 5){
            // Update line information in the frame
            s_lines_3d.push_back(P.leftCols(index));
            cov_G.push_back(line_covariance);
            frame_eigen_values.push_back(eigen_values);
            frame_eigen_vectors.push_back(eigen_vectors);
        }
    }
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

    // populate image dimensions
    im_wd = rgb_image1.cols; im_ht = rgb_image1.rows;

    // Make ransac object for culling outliers later
    pointRefine = new Ransac(10, 1);

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

    // sample lines in image
    SampleIndices(img1_lines, sampled_lines_2d_im1);
    SampleIndices(img2_lines, sampled_lines_2d_im2);

    // Reproject left and right image points to 3D
    Reproject(depth_image1, sampled_lines_2d_im1, points_3d_im1, cov_G_im1, cov_eig_values_im1, cov_eig_vectors_im1);
    Reproject(depth_image2, sampled_lines_2d_im2, points_3d_im2, cov_G_im2, cov_eig_values_im2, cov_eig_vectors_im2);

    // cull outlier points
    for(int i=0; i < points_3d_im1.size(); ++i){
        rsac_points_3d_im1.push_back(pointRefine->removeOutlierPoints(points_3d_im1[i], cov_eig_values_im1[i], cov_eig_vectors_im1[i]));
    }

    for(int i=0; i < points_3d_im2.size(); ++i){
        rsac_points_3d_im2.push_back(pointRefine->removeOutlierPoints(points_3d_im2[i], cov_eig_values_im2[i], cov_eig_vectors_im2[i]));
    }

}
