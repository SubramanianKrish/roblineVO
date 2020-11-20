/*

This runs the full visual odometry system.

Changelog:
    awadhut - 10/02 - Initial commit
    subbu   - 10/02 - Adding Eigen, camera plotter 3D toy
    subbu   - 10/08 - Adding data set parsing
    subbu   - 10/20 - adding reprojector
    subbu   - 11/19 - 3d threaded viewer
*/

// C++ Standard headers
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <thread>
#include <pangolin/pangolin.h>

// Custom headers
#include "frame.h"
#include "utils.h"
#include "line.h"
#include "viewer.h"

using Eigen::MatrixXd;
using namespace std;
using namespace utils;

int main(int argc, char* argv[])
{
  Eigen::initParallel();

  // Start processing the data
  std::cout << "Starting system .." << std::endl;
  cout << "Loading data ..." << endl;
  
  string data_dir     = "../data/rgbd_dataset_freiburg1_xyz/";
  string synched_file = "synched_data.txt";

  string window_name = "ROBust LINE Visual Odometry";;
  viewer* robline_viewer = new viewer(window_name);
  std::thread render_thread(&viewer::run, robline_viewer);

  string image_pair_path, rgb_path, depth_path, rgb_time, depth_time;
  ifstream infile(data_dir + synched_file);

  while(getline(infile, image_pair_path)){
    std::stringstream linestream(image_pair_path);
    linestream >> rgb_time >> rgb_path >> depth_time >> depth_path;
    
    cv::Mat rgb_img   = ReadImage(data_dir + rgb_path);
    cv::Mat depth_img = ReadImage(data_dir + depth_path);

    // Make a frame and populate information internally
    Frame current_frame(rgb_img, depth_img);

    DisplayImage(current_frame.lines->edge_image);

    robline_viewer->updateCurrentFrame(&current_frame);
    // Process two frame information here

    cv::waitKey(0);
  }

  // close file
  infile.close();
  
  // Close viewer completely
  while(!robline_viewer->hasViewerStopped())
    robline_viewer->requestViewerStop();
  // Kill thread
  render_thread.join();
  pangolin::DestroyWindow(window_name);

  return 0;
}
