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
#include <opencv2/core/eigen.hpp>

// Custom headers
#include "frame.h"
#include "utils.h"
#include "line.h"
#include "viewer.h"
#include "ransac.h"

int main(int argc, char* argv[])
{
  Eigen::initParallel();

  // Start processing the data
  std::cout << "Starting system .." << std::endl;
  cout << "Loading data ..." << endl;
  
  // Loading the data (TUMfr1)
  string data_dir     = "../data/rgbd_dataset_freiburg1_xyz/";
  string synched_file = "synched_data.txt";
  string output_file  = "output_poses_full_run.txt";

  string window_name = "ROBust LINE Visual Odometry";
  struct system_poses *robline = new struct system_poses;
  
  // ugly, bad, very very bad - fix later [Why? Don't want vector to be empty. So populate here instead of first image]
  cv::Mat world_pose = cv::Mat::eye(4, 4, CV_64F);
  robline->relative_poses.push_back(world_pose);
  robline->global_poses.push_back(world_pose);

  viewer* robline_viewer = new viewer(window_name, robline);
  
  std::thread render_thread(&viewer::run, robline_viewer);

  string image_pair_path, rgb_path, depth_path, rgb_time, depth_time;
  ifstream infile(data_dir + synched_file);
  ofstream outfile(output_file);

  bool isFirst = true;
  cv::Mat previous_rgb, previous_depth;

  // Vector to hold objects of FramePair class
  // Each object contains lines found in both images and matches between the frames
  std::vector<FramePair*> pairs;
  
  int i=0;
  while(getline(infile, image_pair_path)){
    
    // Sample every 4 frames
    // if(i++%1) continue;

    // if(i > 100) break;

    std::stringstream linestream(image_pair_path);
    linestream >> rgb_time >> rgb_path >> depth_time >> depth_path;
    
    cv::Mat rgb_img   = utils::ReadImage(data_dir + rgb_path, true);
    cv::Mat depth_img = utils::ReadImage(data_dir + depth_path, false);
    if (isFirst)
    {
      previous_rgb = rgb_img;
      previous_depth = depth_img;
      isFirst = false;
    }
    
    else
    { 
      cout << "Starting pair with im2_rgb: " << data_dir + rgb_path << endl;
      // Detect lines, match lines between frames, sample 2d line, reproject to 3d, remove outliers
      FramePair* fpair = new FramePair(previous_rgb, previous_depth, rgb_img, depth_img);
      // pairs.push_back(fpair);
      
      cout << "Frame pair has been successfully created" << endl;

      utils::DrawSampledLines2D(fpair->rgb_image1, fpair->sampled_lines_2d_im1);
      // utils::DrawSampledLines2D(fpair->depth_image1, fpair->sampled_lines_2d_im1);
      utils::DisplayImage(fpair->rgb_image1);

      cout << "Have asked the 2d sampled drawing function" << endl;

      // Update system poses
      cv::Mat current_camera_pose = utils::MakeCameraPose(fpair->R_optim, fpair->t_optim);
      if(fpair->t_optim.lpNorm<Eigen::Infinity>() > 0.4) continue;
      cv::Mat current_global_pose = current_camera_pose*robline->global_poses.back();

      // Lock viewer mtx
      std::unique_lock<std::mutex> curPoseLock(robline_viewer->viewerPosesMtx);
      robline->relative_poses.push_back(current_camera_pose);
      robline->global_poses.push_back(current_global_pose);
      curPoseLock.unlock();
      // Unlock viewer mtx

      // Update the current vieweing frame
      robline_viewer->updateCurrentFrame(fpair);

      cout << "Updated the viewer content!" << endl;

      cv::waitKey(0);
      
      // Update previous frame
      previous_rgb = rgb_img;
      previous_depth = depth_img;
      cout << "updated prev images" << endl;

      // generate required eigen data for saving output
      Eigen::Matrix4d output_Rt;
      cv::cv2eigen(current_global_pose, output_Rt);
      cout << output_Rt << endl;
      cout << current_global_pose << endl;
      Eigen::Quaterniond q(output_Rt.block<3,3>(0,0).transpose());

      // write to output file
      outfile << rgb_time << " " << output_Rt.block<1,3>(3,0) << " " << 
              q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
  }

  // close the output file
  outfile.close();

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
