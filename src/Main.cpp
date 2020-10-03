#include <iostream>
#include <Eigen/Dense>
#include "Utils.h"

using Eigen::MatrixXd;
 
int main()
{
  cv::Mat im1;
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  im1 = read_image("/home/awadhut/geovis/project/data/test.jpg");
  draw_line(im1, 2, 20, 2, 20);
  display_image(im1);
}
