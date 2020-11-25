cmake_minimum_required(VERSION 3.5)

# Make sure that the project is not built in the root directory and is built inside /build folder
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Do not build in-source. Please remove CMakeCache.txt and the CmakeFiles/ directory. Then build out-of source")
endif()

# Set C++ Standard
set(CMAKE_CXX_STANDARD 11)

# Create any folders if not present
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Make the library directory (Can hold .so or .a files)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# Include OpenCV and Eigen
find_package(OpenCV 3.4.9 QUIET)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 is not found. Please check readme")
endif()
# Need Eigen 3 and above for the all variable
find_package(Eigen3 3.3.8 REQUIRED)
message("This is the path to eigen: ${EIGEN3_INCLUDE_DIRS}")
find_package(Pangolin REQUIRED)

# Add libraries - tells CMake to look for headers here when compiling
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../LBD_and_LineMatching/  
    )

    # Create a static library (compiled and linked completely -> One big executable)
    add_library(${PROJECT_NAME} STATIC 
    ${CMAKE_SOURCE_DIR}/src/utils.cpp
    ${CMAKE_SOURCE_DIR}/src/line.cpp
    ${CMAKE_SOURCE_DIR}/src/frame.cpp
    ${CMAKE_SOURCE_DIR}/src/viewer.cpp
    )
    
    LINK_DIRECTORIES(/home/pitcrew/geovis/LBD_and_LineMatching/build/)
    # Link external libraries (opencv, eigen and our custom library)
    target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        LineMatchingLib
    )

# Add main executable file
add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(main PUBLIC ${PROJECT_NAME} )

# Add unit_test executables
add_executable(viewer ${CMAKE_CURRENT_SOURCE_DIR}/test_viewer.cpp)
target_link_libraries(viewer PUBLIC ${PROJECT_NAME})

add_executable(eigen_test ${CMAKE_CURRENT_SOURCE_DIR}/eigen_test.cpp)
target_link_libraries(eigen_test PUBLIC ${PROJECT_NAME})

add_executable(lsd_test ${CMAKE_CURRENT_SOURCE_DIR}/lsd_test.cpp)
target_link_libraries(lsd_test PUBLIC ${PROJECT_NAME})
