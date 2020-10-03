cmake_minimum_required(VERSION 3.5)

# Make sure that the project is not built in the root directory and is built inside /build folder
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Do not build in-source. Please remove CMakeCache.txt and the CmakeFiles/ directory. Then build out-of source")
endif()

# Set C++ Standard
set(CMAKE_CXX_STANDARD 11)

# Create any folders if not present
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# Add libraries
include_directories(${CMAKE_SOURCE_DIR}/include)


# Create a static library (compiled and linked completely)
add_library(utils STATIC ${CMAKE_SOURCE_DIR}/src/utils.cpp)


# Include OpenCV and Eigen
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# Add main executable file
add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(main PUBLIC utils ${OpenCV_LIBS} ${EIGEN3_LIBS})
