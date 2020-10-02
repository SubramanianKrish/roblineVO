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

## Build instruction
> $ mkdir build && cd build  
> $ cmake ..  
> $ make -j2  


### References:
Pangolin utils: https://github.com/uoip/pangolin/blob/master/python/contrib.hpp [lines, points, cameras]
