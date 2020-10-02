/*

This runs the full visual odometry system.

Changelog:
    awadhut - 10/02 - Initial commit
    subbu   - 10/02 - subbu - 10/02 - Adding Eigen, camera plotter 3D toy 
*/

// C++ Standard headers
#include <iostream>
#include <Eigen/Dense>

// Custom headers
#include "frame.h"
#include "utils.h"

using Eigen::MatrixXd;
 
int main()
{
  std::cout << "Starting system now .." << std::endl;

  cv::Mat im1;
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  im1 = ReadImage("../data/test.jpg");
  Frame test_frame(image);

  DrawLine(im1, 2, 2, 200, 200);
  DisplayImage(im1);
  
  return 0;
}
