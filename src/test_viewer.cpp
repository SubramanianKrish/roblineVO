#include <iostream>
#include <pangolin/pangolin.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace pangolin;

// Code reference: https://github.com/uoip/pangolin/blob/master/python/contrib.hpp
void DrawCamera(cv::Mat camera, float w=1.0, float h_ratio=0.75, float z_ratio=0.6) {
    float h = w * h_ratio;
    float z = w * z_ratio;

    glPushMatrix();
    // glMultMatrixd(r.data(0, 0));
    // glMultTransposeMatrixd(r.data(0, 0));

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

int main(){
    cout << "testing 3D viewer" << endl;

    // Needs to have 16 entries which would 
    float camera_pose_array[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    cv::Mat camera_pose = cv::Mat(4, 4, CV_32F, camera_pose_array);
    cout << camera_pose.rows << " " << camera_pose.cols << endl;

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

        // Render OpenGL Cube
        DrawCamera(camera_pose);

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    
}
