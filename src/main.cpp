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
#include "line.h"

using Eigen::MatrixXd;
using namespace std;
using namespace utils;


int main(int argc, char* argv[])
{
  std::cout << "Starting system .." << std::endl;
  cout << "Loading data ..." << endl;
  
  // Loading the data (TUMfr1)
  string data_dir     = "../data/rgbd_dataset_freiburg1_xyz/";
  string synched_file = "synched_data.txt";
  string image_pair_path, rgb_path, depth_path, rgb_time, depth_time;
  ifstream infile(data_dir + synched_file);

  // Line object will probably not need it here. Check later
  vo_line line_obj;

  // Vector to hold consecutive frames in the dataset
  std::vector<Frame> frames;

  // Vector to hold objects of FramePair class
  // Each object contains lines found in both images and matches between the frames
  std::vector<FramePair> pairs;
  
  while(getline(infile, image_pair_path)){
    std::stringstream linestream(image_pair_path);
    linestream >> rgb_time >> rgb_path >> depth_time >> depth_path;
    
    cv::Mat rgb_img   = ReadImage(data_dir + rgb_path);
    cv::Mat depth_img = ReadImage(data_dir + depth_path);
    
    Frame frame(rgb_img, depth_img);

    frames.push_back(frame);

    if (frames.size() == 2){
      // Create a pair object and store it in a vector for future access
      FramePair pair(frames[0].rgb_image, frames[0].depth_image, rgb_img, depth_img);
      pairs.push_back(pair);
      
      // Remove the first frame from the vector
      frames.erase(frames.begin());
    }

    // DisplayDualImage(rgb_img, depth_img);

    // Eigen::Matrix4i linesP; // will hold the results of the detection
    
    // cv::Mat hough_img = DrawHoughLinesP(rgb_img, linesP);
  
    // std::vector<std::vector<cv::Point2i>> sampled_lines;

    // sampled_lines = line_obj.SampleIndices(linesP, 480, 640);
    
    // DrawSampledLines2D(rgb_img, sampled_lines);

  }
  
  // for(auto pair: pairs){
  //   std::cout << pair.pstruct.linesInLeft.size() << std::endl;
  //   DisplayDualImage(pair.rgb_image1, pair.rgb_image2);
  // }

  // close file
  infile.close();

  return 0;
}
