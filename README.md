# roblineVO
Robust Line based Visual odometry using RGB-D images

## System requirements
1. CMake version > 3.0 [Check with > cmake --version]

2. (Developed on) OpenCV 3.4.9
    + To download please use: https://github.com/opencv/opencv/archive/3.4.9.zip   
    + To install, refer https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html   
        Our build configuration: CMAKE_BUILD_TYPE=Release, OPENCV_GENERATE_PKGCONFIG=ON  
    + Check installation using pkg-config --modversion opencv  

3. Eigen 3.0 or higher [Check with > pkg-config --modversion eigen3 ]  

4. Pangolin [Download here: https://github.com/stevenlovegrove/Pangolin]  
    + OpenGL  
    > sudo apt install libgl1-mesa-dev  
    + Glew  
    > sudo apt install libglew-dev  

5. Python 2.7 [Optional]  
    For running the associate script to time synchronize RGB and Depth images of the TUM Dataset

## Build instruction
> $ mkdir build && cd build  
> $ cmake ..  
> $ make -j2  

## Dataset
We use the TUM fr1 sequence for developmental testing [https://vision.in.tum.de/data/datasets/rgbd-dataset/download]  
Download the fr1/xyz sequence and extract it  
Place the extracted folder in the data directory  
P.S: Run the associate script in tools to time synchronize the rgb and depth images  

<code> $ python associate.py ../data/rgbd_dataset_freiburg1_xyz/rgb.txt ../data/rgbd_dataset_freiburg1_xyz/depth.txt </code>  
This should've created the synched_data.txt file in the fr1 data directory

### References:
Pangolin utils: https://github.com/uoip/pangolin/blob/master/python/contrib.hpp [lines, points, cameras]

### Isses that pop up once in a while:
If you get librealsense2.so is missing for pangolin, clear build and rebuild Pangolin. Temporary hacky fix.
