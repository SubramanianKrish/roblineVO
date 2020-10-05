#include "utils.h"

cv::Mat ReadImage(std::string image_path){
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    return img;
}

void DisplayImage(cv::Mat img){
    if ( !img.data )                                        // check validity of image
    {
        printf("No image data \n");
        return;
    }
    cv::namedWindow("Display Image");
    cv::imshow("Display Image", img);
    cv::waitKey(0);
}

void DrawLine(cv::Mat img, int x1, int y1, int x2, int y2){
    cv::Point start = cv::Point(x1, y1);
    cv::Point end = cv::Point(x2, y2);
    int thickness = 2;
    int lineType = cv::LINE_8;                              // the line is an 8-connected line
    cv::line(img, start, end, cv::Scalar( 0, 0, 0 ),        // https://en.wikipedia.org/wiki/Pixel_connectivity
            thickness, lineType );
}

// Code reference: https://github.com/uoip/pangolin/blob/master/python/contrib.hpp
void DrawSingleCamera(const cv::Mat& camera_pose, float w, float h_ratio, float z_ratio) {
    // Tunes the length to width ratio of the rectangle
    float h = w * h_ratio;
    // Tunes the height to scale ratio
    float z = w * z_ratio;

    // Note: gl uses column major representation for transformation matrices.
    // Hence we're using the mulTransposeMetrix version
    glPushMatrix();
    glMultTransposeMatrixf(camera_pose.ptr<GLfloat>(0));

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void DisplayDualImage(const cv::Mat& im1, const cv::Mat& im2){
    cv::Mat concatenated_image;
    cv::hconcat(im1, im2, concatenated_image);
    DisplayImage(concatenated_image);
}
