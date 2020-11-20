// C++ Standard headers
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

// Custom headers
#include "frame.h"
#include "utils.h"
#include "line.h"

using namespace std;
using namespace utils;
using namespace Eigen;


int main(int argc, char* argv[])
{ 
    // float array[28];
    // for(int i = 0; i < 28; ++i) array[i] = i;

    // vo_line line_obj;
    // string data_dir     = "../data/lines.png";
    // cv::Mat rgb_img   = ReadImage(data_dir);
    // cv::Mat edge_image = line_obj.GetEdgeImage(rgb_img, 50, 150, 3, true);
    
    // Eigen::MatrixXi linesP; // will hold the results of the detection
    // linesP = line_obj.GetHoughLinesP(edge_image, 50, 50, 10);
    // // std::cout << linesP.transpose()*linesP << std::endl;
    // cv::Mat img_with_lines = line_obj.DrawHoughLinesP(rgb_img, linesP);
    // std::vector<std::vector<cv::Point2i>> sampled_lines = line_obj.SampleIndices(linesP, 480, 640);
    // DisplayImage(img_with_lines);
    
    // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    // int min = 0, max = 10;
    // // Testing slicing
    // Eigen::MatrixXd my_mat = Eigen::MatrixXd::Random(7,7);
    // cout << my_mat.format(CleanFmt) << endl;

    // Eigen::MatrixXi index(5);
    // index << 3,5,7,3,6;
    // cout << my_mat(Eigen::all, Eigen::all) << endl;

    Eigen::Matrix<double, 5, 5> a;
    for(int i=0; i<5; ++i){
        for(int j=0; j<5; ++j){
            a(i,j) = i*5 + j;
        }
    }

    Eigen::Matrix<double,3,2> indices;

    indices = (Eigen::Matrix<double, 3, 2>() << 2,3,
                                                4,4,
                                                1,2).finished();
    cout << a << endl;
    cout << indices << endl;

    cout << a(Eigen::all, indices(Eigen::all, 0)) << endl;

    return 0;
}
