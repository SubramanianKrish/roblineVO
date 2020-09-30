# roblineVO
Robust Line based Visual odometry using RGB-D images

## System requirements
1. CMake version > 3.0 [Check with cmake --version]

2. (Developed on) OpenCV 3.4.9
    + a. To download please use: https://github.com/opencv/opencv/archive/3.4.9.zip
    + b. To install, refer https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
        Our build configuration: CMAKE_BUILD_TYPE=Release, OPENCV_GENERATE_PKGCONFIG=ON
    + c. Check installation using pkg-config --modversion opencv

3. Eigen 3.0 or higher [Check with pkg-config --modversion eigen3]
