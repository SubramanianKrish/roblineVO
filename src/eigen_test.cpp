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
    float array[28];
    for(int i = 0; i < 28; ++i) array[i] = i;

    vo_line line_obj;
    string data_dir     = "../data/lines.png";

    cv::Mat rgb_img   = ReadImage(data_dir);
    cv::Mat edge_image = line_obj.GetEdgeImage(rgb_img, 50, 150, 3, true);
    
    Eigen::MatrixXi linesP; // will hold the results of the detection
    linesP = line_obj.GetHoughLinesP(edge_image, 50, 50, 10);

    cv::Mat img_with_lines = line_obj.DrawHoughLinesP(rgb_img, linesP);
    std::vector<std::vector<cv::Point2i>> sampled_lines = line_obj.SampleIndices(linesP, 480, 640);

    DisplayImage(img_with_lines);
    
    return 0;
}
