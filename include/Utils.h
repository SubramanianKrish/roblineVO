#include <opencv2/opencv.hpp>

void DisplayImage(cv::Mat img);

cv::Mat ReadImage(std::string image_path);

void DrawLine(cv::Mat img, int x1, int y1, int x2, int y2);     // draw line on image given the coordinates of endpoints