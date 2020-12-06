/*
Main viewer class for setting up pangolin based rendering system

Date: 19 Nov 2020

Changelog:
    subbu - 11/19 - Initial commit
*/
# pragma once

#include <pangolin/pangolin.h>
#include <string>
#include <mutex>

#include "frame.h"
#include "utils.h"

class viewer{
    public:
        // constructor for the viewer
        viewer(const std::string& window_name, struct system_poses* ptr_to_poses);

        // main render loop
        void run();

        // Setting the stop signal from main thread
        void requestViewerStop();
        // Checking the stop condition inside render loop
        bool hasViewerStopped();

        // Update the frame which we want to plot
        void updateCurrentFrame(FramePair* input_frame);

        // Mutex to guard data that needs to be plotted (current_frame)
        std::mutex viewerVarsMtx;

        // Mutex to guard the poses data
        std::mutex viewerPosesMtx;
        
    private:
        // Pointer to frame whose data we want to plot
        FramePair* current_frame;

        // Pointer to the system of poses
        struct system_poses* camera_poses;

        // To close viewer at end of processing
        bool stopViewer;
        std::mutex stopViewerMtx;


        std::string window_name;
};
