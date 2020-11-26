#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

#include "utils.h"

using namespace std;
using namespace pangolin;
using namespace utils;

int main(){
    cout << "testing 3D viewer" << endl;

    // Needs to have 16 entries which would make up the Transformation matrix
    float camera_pose_array[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    float camera_pose_trans_array[16] = {0.7071068, 0, 0.7071068, 0, 0, 1, 0, 0, -0.7071068, 0,  0.7071068, 0, 0, 0, 0, 1};
    
    cv::Mat camera_pose_origin = cv::Mat(4, 4, CV_32F, camera_pose_array);
    cv::Mat camera_pose_translated = cv::Mat(4, 4, CV_32F, camera_pose_trans_array);
    
    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        DrawCoordinates();
        // Draw test camera
        DrawSingleCamera(camera_pose_origin);
        // DrawSingleCamera(camera_pose_translated);

        // Testing 3D line plot
        cv::Point3d x = cv::Point3d(1.0, 1.0, 1.0);
        cv::Point3d y = cv::Point3d(-1.0, -1.0, -1.0);
        std::vector<std::vector<cv::Point3d>> lines = {{x,y}};
        std::vector<cv::Point3d> points = {x,y};
        // DrawLines(lines);
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }    
}
