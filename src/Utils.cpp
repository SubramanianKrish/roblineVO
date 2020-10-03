#include <stdio.h>
#include "Utils.h"

cv::Mat read_image(std::string image_path){
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    return img;
}

void display_image(cv::Mat img){
    if ( !img.data )
    {
        printf("No image data \n");
        return;
    }
    cv::namedWindow("Display Image");
    cv::imshow("Display Image", img);
    cv::waitKey(0);
}



void draw_line(cv::Mat img, int x1, int y1, int x2, int y2){
    cv::Point start = cv::Point(x1, y1);
    cv::Point end = cv::Point(x2, y2);
    int thickness = 2;
    int lineType = cv::LINE_8;
    cv::line( img,
        start,
        end,
        cv::Scalar( 0, 0, 0 ),
        thickness,
        lineType );
}