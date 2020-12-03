#include "viewer.h"
#include "frame.h"
#include "utils.h"

#include <unistd.h>
#include <iostream>

viewer::viewer(const string& window_name) : window_name(window_name), current_frame(NULL), stopViewer(false) {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void viewer::run(){
    pangolin::BindToContext(window_name);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    
    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        utils::DrawCoordinates();

        // CRITICAL SECTION
        std::unique_lock<std::mutex> curFrameLock(viewerVarsMtx);

        if(current_frame != NULL and current_frame->points_3d_im1.size()!= 0){
            utils::DrawPoints3D(current_frame->rsac_points_3d_im1, {1.0, 1.0, 0.0}, 3);
        }
        
        curFrameLock.unlock();

        // Swap frames and Process Events
        pangolin::FinishFrame();
        
        if(hasViewerStopped()){
            break;
        }
        // End of CRITICAL SECTION
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void viewer::updateCurrentFrame(FramePair* input_frame){
    std::unique_lock<std::mutex> curFrameLock(viewerVarsMtx);
    current_frame = input_frame;
}

void viewer::requestViewerStop(){
    std::unique_lock<std::mutex> stopLock(stopViewerMtx);
    stopViewer = true;
}

bool viewer::hasViewerStopped(){
    std::unique_lock<std::mutex> stopLock(stopViewerMtx);
    return stopViewer; // assuming once stopViewer is True, the viewer has stopped
}
