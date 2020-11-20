#include <vector>
#include <Eigen/Dense>
#include <cmath>

#include "utils.h"

namespace utils{
    
    // Read in the image
    // <TODO/possible BUG> We read in depth image as color. Read as 1D image?
    cv::Mat ReadImage(const std::string& image_path){
        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        return img;
    }
    
    // Used to display the 2D image
    void DisplayImage(const cv::Mat& img){
        if ( !img.data )                                        // check validity of image
        {
            printf("No image data \n");
            return;
        }
        cv::namedWindow("Display Image", CV_WINDOW_NORMAL);
        cv::imshow("Display Image", img);
        cv::waitKey(10);
    }

    // Display two images side by side
    void DisplayDualImage(const cv::Mat& im1, const cv::Mat& im2){
        cv::Mat concatenated_image;
        cv::hconcat(im1, im2, concatenated_image);
        DisplayImage(concatenated_image);
    }

    // Helper to draw lines on 2D images
    void DrawLine2D(const cv::Mat& img, const int& x1, const int& y1, const int& x2, const int& y2){
        cv::Point start = cv::Point(x1, y1);
        cv::Point end = cv::Point(x2, y2);
        int thickness = 2;
        int lineType = cv::LINE_8;                              // the line is an 8-connected line
        cv::line(img, start, end, cv::Scalar( 0, 0, 0 ),        // https://en.wikipedia.org/wiki/Pixel_connectivity
                thickness, lineType );
    }

    // Helper to draw houghlines
    cv::Mat DrawHoughLinesP(cv::Mat img, Eigen::MatrixXi linesP){
    for(std::size_t i = 0; i < linesP.rows(); i++)
    {
        Eigen::Vector4i l = linesP.row(i);
        line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    return img;
}

    // Draw 3D lines connecting two 3D points
    void DrawLines3D(const std::vector<std::vector<cv::Point3d>>& lines) {
        glBegin(GL_LINES);
        for (auto it = lines.begin(); it != lines.end(); it++) {
            double x1 = (*it)[0].x, y1 = (*it)[0].y, z1 = (*it)[0].z;
            double x2 = (*it)[1].x, y2 = (*it)[1].y, z2 = (*it)[1].z;
            glVertex3d(x1, y1, z1);
            glVertex3d(x2, y2, z2);
        }
        glEnd();
    }

    // Draw 3D points
    void DrawPoints3D(const Eigen::MatrixXd& points, const std::vector<double>& color, const float& point_size = 0) {
        if(point_size > 0) {
            glPointSize(point_size);
        }
        glBegin(GL_POINTS);
        glColor3f(color[0], color[1], color[2]);
        int size = points.cols();
        for (int i=0; i<size; ++i){
            glVertex3d(points(0,i), points(1,i), points(2,i));
        }
        glEnd();
    }
    
    // Draw points sampled on lines in 2D
    void DrawSampledLines2D(const cv::Mat& img, const std::vector<std::vector<cv::Point2i>>& sampled_lines)
    {   
        // Ensure auto doesn't create copies. So pass by reference
        for(auto& line: sampled_lines){
            for(auto& point: line){
                cv::circle(img, point, 3, CV_RGB(255,0,0), 1);
            }
        }
    }

    // Code reference: https://github.com/uoip/pangolin/blob/master/python/contrib.hpp
    void DrawSingleCamera(const cv::Mat& camera_pose, const float& w, const float& h_ratio, const float& z_ratio) {
        // Tunes the length to width ratio of the rectangle
        float h = w * h_ratio;
        // Tunes the height to scale ratio
        float z = w * z_ratio;

        // Note: gl uses column major representation for transformation matrices.
        // Hence we're using the mulTransposeMatrix version
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

    void DrawCoordinates(){
 
        // draw some lines
        glColor3f(1.0,0.0,0.0); // red x
        glBegin(GL_LINES);
        // x axis
    
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

}
