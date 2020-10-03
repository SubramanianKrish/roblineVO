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

  im1 = ReadImage("../data/test.jpg");
  DrawLine(im1, 2, 2, 200, 200);
  DisplayImage(im1);
  return 0;
}
