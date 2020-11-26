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
  
  // Loading the data (TUMfr1)
  string data_dir     = "../data/rgbd_dataset_freiburg1_xyz/";
  string synched_file = "synched_data.txt";

  string window_name = "ROBust LINE Visual Odometry";;
  viewer* robline_viewer = new viewer(window_name);
  std::thread render_thread(&viewer::run, robline_viewer);

  string image_pair_path, rgb_path, depth_path, rgb_time, depth_time;
  ifstream infile(data_dir + synched_file);

  bool isFirst = true;
  cv::Mat previous_rgb, previous_depth;

  // Line object will probably not need it here. Check later
  // vo_line line_obj();

  // Vector to hold consecutive frames in the dataset
  std::vector<Frame> frames;

  // Vector to hold objects of FramePair class
  // Each object contains lines found in both images and matches between the frames
  std::vector<FramePair*> pairs;
  
  while(getline(infile, image_pair_path)){
    std::stringstream linestream(image_pair_path);
    linestream >> rgb_time >> rgb_path >> depth_time >> depth_path;
    
    cv::Mat rgb_img   = ReadImage(data_dir + rgb_path);
    cv::Mat depth_img = ReadImage(data_dir + depth_path);

    // Make a frame and populate information internally
    // Frame* current_frame = new Frame(rgb_img, depth_img);

    // DisplayImage((*current_frame).lines->edge_image);

    // robline_viewer->updateCurrentFrame(current_frame);
    // // Process two frame information here

    // cv::waitKey(10);
    
    if (isFirst)
    {
      previous_rgb = rgb_img;
      previous_depth = depth_img;
      isFirst = false;
    }
    else
    {
      FramePair* fpair = new FramePair(previous_rgb, previous_depth, rgb_img, depth_img);
      pairs.push_back(fpair);
      // DisplayDualImage((*fpair).rgb_image1, (*fpair).rgb_image2);
      // // DisplayDualImage(previous_rgb, rgb_img);
      // cv::waitKey(1);
      previous_rgb = rgb_img;
      previous_depth = depth_img;
    }



    // Frame frame(rgb_img, depth_img);

    // frames.push_back(frame);

    // if (frames.size() == 2){
    //   // Create a pair object and store it in a vector for future access
    //   FramePair pair(frames[0].rgb_image, frames[0].depth_image, rgb_img, depth_img);
    //   pairs.push_back(pair);
      
    //   // Remove the first frame from the vector
    //   frames.erase(frames.begin());
    // }


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
  
  // Close viewer completely
  while(!robline_viewer->hasViewerStopped())
    robline_viewer->requestViewerStop();
  // Kill thread
  render_thread.join();
  pangolin::DestroyWindow(window_name);

  return 0;
}
