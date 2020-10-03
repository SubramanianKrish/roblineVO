#include <opencv2/opencv.hpp>

void display_image(cv::Mat img);

cv::Mat read_image(std::string image_path);

void draw_line(cv::Mat img, int x1, int y1, int x2, int y2);