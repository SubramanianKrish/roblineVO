#include <vector>
#include <Eigen/Dense>
#include <cmath>

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
    // cv::waitKey(0);
}

cv::Mat GetEdgeImage(cv::Mat& img, int lowThreshold = 50, int highThreshold = 150, int kernel_size = 3, bool enable_blur = false){
    cv::Mat smooth_image, gray_image, edge_image;
    if (enable_blur){
        cv::blur(img, smooth_image, cv::Size(5, 5), cv::Point(-1,-1) );
    }
    else{
        smooth_image = img;
    }
    cv::cvtColor(smooth_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Canny(gray_image, edge_image, lowThreshold, highThreshold, kernel_size);
    return edge_image;
}

std::vector<cv::Vec4i> GetHoughLinesP(cv::Mat& edge_image, int thresh = 50, int minLen = 50, int maxGap = 10){
    std::vector<cv::Vec4i> linesP;
    cv::HoughLinesP(edge_image, linesP, 1, CV_PI/180, thresh, minLen, maxGap);
    return linesP;
}

cv::Mat DrawHoughLinesP(cv::Mat img, std::vector<cv::Vec4i> linesP){
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    return img;
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

void DrawLines(std::vector<std::vector<cv::Point3d>> lines) {
    glBegin(GL_LINES);
    for (auto it = lines.begin(); it != lines.end(); it++) {
        double x1 = (*it)[0].x, y1 = (*it)[0].y, z1 = (*it)[0].z;
        double x2 = (*it)[1].x, y2 = (*it)[1].y, z2 = (*it)[1].z;
        glVertex3d(x1, y1, z1);
        glVertex3d(x2, y2, z2);
    }
    glEnd();
}

void drawCoordinates(){
 
    // draw some lines
    glColor3f(1.0,0.0,0.0); // red x
    glBegin(GL_LINES);
    // x aix
 
    glVertex3f(0.0, 0.0f, 0.0f);
    glVertex3f(3.0, 0.0f, 0.0f);
 
    glVertex3f(3.0, 0.0f, 0.0f);
    glVertex3f(2.0, 1.0f, 0.0f);
 
    glVertex3f(3.0, 0.0f, 0.0f);
    glVertex3f(2.0, -1.0f, 0.0f);
    glEnd();
 
    // y 
    glColor3f(0.0,1.0,0.0); // green y
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f, 0.0f);
    glVertex3f(0.0, 3.0f, 0.0f);
 
    glVertex3f(0.0, 3.0f, 0.0f);
    glVertex3f(1.0, 2.0f, 0.0f);
 
    glVertex3f(0.0, 3.0f, 0.0f);
    glVertex3f(-1.0, 2.0f, 0.0f);
    glEnd();
 
    // z 
    glColor3f(0.0,0.0,1.0); // blue z
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f ,0.0f );
    glVertex3f(0.0, 0.0f ,3.0f );
 
 
    glVertex3f(0.0, 0.0f ,3.0f );
    glVertex3f(0.0, 1.0f ,2.0f );
 
    glVertex3f(0.0, 0.0f ,3.0f );
    glVertex3f(0.0, -1.0f ,2.0f );

    glColor3f(1.0,1.0,1.0);
    glEnd();
 
}

void DrawPoints(std::vector<cv::Point3d> points, std::vector<double> color, float point_size = 0) {
    if(point_size > 0) {
        glPointSize(point_size);
    }
    glBegin(GL_POINTS);
    glColor3f(color[0], color[1], color[2]);
    for (auto it = points.begin(); it != points.end(); it++){
        double x = (*it).x, y = (*it).y, z = (*it).z;
        glVertex3d(x, y, z);
    }
    glEnd();
}

std::vector<std::vector<int>> SampleIndices(const std::vector<std::vector<int>>& lines, const int& ht, const int& wd){
    /*
    Input: lines vector expected to be of form nx4 with each entry being [x1,y1,x2,y2] 
    where x_i, y_i denote the end points of the lines
    */
    for(auto& line: lines){
        Eigen::Vector2f p1(line[0], line[1]), p2(line[2], line[3]);
        float dist = (p1-p2).norm();
        float n_samples = std::min(100, int(dist));

        // ref: https://stackoverflow.com/questions/28018147/emgucv-get-coordinates-of-pixels-in-a-line-between-two-points
        int x0 = line[0], y0 = line[1], x1 = line[2], y1 = line[3];
        int dx = std::abs(x1- x0), dy = std::abs(y1-y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        std::vector<std::vector<int>> points;

        if (n_samples < 100){
            // Bresenham's algorithm
            while(true){
                std::vector<int> point{x0,y0};
                points.push_back(point);

                if(x0 == x1 && y0 == y1) break;
                int e2 = 2*err;
                if(e2 > -dy){
                    err = err - dy;
                    x0 = x0 + sx;
                }
                if(e2 < dx){
                    err = err + dx;
                    y0 = y0 + sy;
                }
            }
        }
        else{
            // Do something else :/
        }
        return points;
    }
}
