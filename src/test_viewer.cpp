#include <iostream>
#include <Eigen/Dense>
#include <opencv/highgui.h>
#include <fstream>
#include <unistd.h>
#include <string>

#include <pangolin/pangolin.h>
#include "utils.h"

using namespace std;
using namespace pangolin;
using namespace Eigen;
using namespace utils;

int main(){
    cout << "testing 3D viewer" << endl;

    // Needs to have 16 entries which would make up the Transformation matrix
    Eigen::Matrix4d origin = Eigen::MatrixXd::Identity(4,4);
    Eigen::Matrix4d trans_x = Eigen::MatrixXd::Identity(4,4);
    trans_x(0,3) = 1;

    Eigen::Matrix4d trans_y = Eigen::MatrixXd::Identity(4,4);
    trans_y(1,3) = -2;

    Eigen::Matrix4d rot_x_trans_z;
    rot_x_trans_z << 1.0000000,  0.0000000,  0.0000000, 0,
                     0.0000000,  0.5253220, -0.8509035, 0,
                     0.0000000,  0.8509035,  0.5253220, 1.5,
                     0, 0, 0, 1;
    std::vector<cv::Mat> poses;

    cv::Mat p1(4,4,CV_64F, origin.data());
    cv::Mat p2(4,4,CV_64F, trans_x.data());
    cv::Mat p3(4,4,CV_64F, trans_y.data());
    cv::Mat p4(4,4,CV_64F, rot_x_trans_z.data());

    poses.push_back(p1);
    poses.push_back(p2);
    poses.push_back(p3);
    poses.push_back(p4);

    std::vector<cv::Mat> gpose;
    gpose.push_back(p1);
    for(int i=1; i<=3; ++i){
        gpose.push_back(poses[i]*gpose.back());
    }

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
    
    ifstream infile("/home/subbu/full_run_output_poses_backup.txt");
    string data, timestamp, gx, gy, gz, qx, qy, qz, qw;
    std::string::size_type sz;

    double x,y,z,rx,ry,rz,rw;
    std::vector<cv::Mat> cameras;
    bool first = true;
    
    while( !pangolin::ShouldQuit() )
    {   
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        DrawCoordinates();
        
        // read data
        getline(infile, data);
        stringstream linestream(data);
        linestream >> timestamp >> gx >> gy >> gz >> qx >> qy >> qz >> qw;

        x = stod(gx, &sz);
        y = stod(gy, &sz);
        z = stod(gz, &sz);
        rx = stod(qx, &sz);
        ry = stod(qy, &sz);
        rz = stod(qz, &sz);
        rw = stod(qw, &sz);

        // cout << x << " " << y << " " << z << " " << rx << " "<< ry << " "<< rz << " " << rw << endl; 
        // Make camera pose
        double quaternion[4] = {rx, ry, rz, rw};
        Quaterniond myquat(quaternion);
        Eigen::Matrix3d R = myquat.toRotationMatrix();
        Eigen::Vector3d t;
        t << x, y, z;
        Eigen::Matrix4d Affine;
        Affine.setIdentity();
        Affine.block<3,3>(0,0) = R;
        Affine.block<3,1>(0,3) = t;
        cv::Mat camera_pose(4,4,CV_64F, Affine.data());
        cout << Affine << endl;
        
        cameras.push_back(camera_pose.clone());
        bool ml;

        // int n = 30;
        // end = cameras.size() < 10? cameras.size(): 10;
        for(int i=0; i<cameras.size(); i+=10){
            // if(i == cameras.size()- 1) glColor3f(1,0,0);
            DrawSingleCamera(cameras[i]);
            if(first){
            cin >> ml;
            first = false;
            } 
        }

        // Draw test camera
        
        usleep(100000); // sleep for 0.3 sec

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    infile.close();
}
