/*

This runs the full visual odometry system.

Changelog:
    awadhut - 10/02 - Initial commit
    subbu   - 10/02 - Adding Eigen, camera plotter 3D toy
    subbu   - 10/08 - Adding data set parsing
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

int main(int argc, char* argv[])
{
  std::cout << "Starting system now .." << std::endl;
  
  cv::Mat im1;
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  im1 = ReadImage("../data/lines.png");
  Frame test_frame(im1);

  DrawLine(im1, 2, 2, 200, 200);
  // DisplayImage(im1);
  DisplayDualImage(im1, im1);

  cv::Mat edge_image = GetEdgeImage(im1, 50, 150, 3, true);

  DisplayImage(edge_image);




  std::vector<cv::Vec4i> linesP; // will hold the results of the detection
  
  linesP = GetHoughLinesP(edge_image, 50, 50, 10);

  cv::Mat hough_img = DrawHoughLinesP(im1, linesP);
  
  DisplayImage(hough_img);
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
    cv::waitKey(10);
  }

  // close file
  infile.close();

  return 0;


}
