/*

This runs the full visual odometry system.

Changelog:
    awadhut - 10/02 - Initial commit
    subbu   - 10/02 - Adding Eigen, camera plotter 3D toy
    subbu   - 10/08 - Adding data set parsing
    subbu   - 10/20 - adding reprojector
*/

// C++ Standard headers
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

// Custom headers
#include "frame.h"
#include "utils.h"

using Eigen::MatrixXd;
using namespace std;
using namespace utils;


int main(int argc, char* argv[])
{
  std::cout << "Starting system .." << std::endl;
  cout << "Loading data ..." << endl;
  
  string data_dir     = "../data/rgbd_dataset_freiburg1_xyz/";
  string synched_file = "synched_data.txt";
  string image_pair_path, rgb_path, depth_path, rgb_time, depth_time;
  ifstream infile(data_dir + synched_file);

  while(getline(infile, image_pair_path)){
    std::stringstream linestream(image_pair_path);
    linestream >> rgb_time >> rgb_path >> depth_time >> depth_path;
    
    cv::Mat rgb_img   = ReadImage(data_dir + rgb_path);
    cv::Mat depth_img = ReadImage(data_dir + depth_path);
    
    DisplayDualImage(rgb_img, depth_img);

    cv::Mat edge_image = GetEdgeImage(rgb_img, 50, 150, 3, true);

    // DisplayImage(edge_image);

    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    
    linesP = GetHoughLinesP(edge_image, 50, 50, 10);

    // cv::Mat hough_img = DrawHoughLinesP(rgb_img, linesP);
  
    // DisplayImage(hough_img);

    std::vector<std::vector<cv::Point2i>> sampled_lines;

    sampled_lines = SampleIndices(linesP, 480, 640);
    
    DrawSampledLines2D(rgb_img, sampled_lines);
    
    // DisplayImage(rgb_img);


    cv::waitKey(10);
  }

  // close file
  infile.close();

  return 0;
}
