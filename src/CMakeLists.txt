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
SET(LAPACKBLAS_DIR "/usr/lib" CACHE PATH "Path to lapack/blas libraries")
SET(LAPACKBLAS_LIB_NAMES "lapack;blas" CACHE STRING "The name of the lapack & blas libraries")

# IF(HAVE_PLASMA)
#  INCLUDE_DIRECTORIES(${PLASMA_DIR}/include ${PLASMA_DIR}/quark)
# ENDIF(HAVE_PLASMA)

# # PLASMA headers in Axb.c should be compiled as C++
# IF(HAVE_PLASMA)
#   SET_SOURCE_FILES_PROPERTIES(Axb.c PROPERTIES LANGUAGE CXX)
# ENDIF(HAVE_PLASMA)

# IF(NEED_F2C)
#   SET(F2C_LIB_NAME f2c CACHE STRING "The name of the f2c or F77/I77 library")
#   # f2c is sometimes equivalent to libF77 & libI77
#   # SET(F2C_LIB_NAME "libF77; libI77" CACHE STRING "The name of the f2c or F77/I77 library")
# ELSE(NEED_F2C)
#   SET(F2C_LIB_NAME "" CACHE STRING "The name of the f2c or F77/I77 library")
# ENDIF(NEED_F2C)

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
    ${CMAKE_SOURCE_DIR}/../levmar-2.6/
    )
    
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../levmar-2.6/build/)
# Create a static library (compiled and linked completely -> One big executable)
add_library(${PROJECT_NAME} STATIC 
${CMAKE_SOURCE_DIR}/src/utils.cpp
${CMAKE_SOURCE_DIR}/src/line.cpp
${CMAKE_SOURCE_DIR}/src/frame.cpp
${CMAKE_SOURCE_DIR}/src/viewer.cpp
${CMAKE_SOURCE_DIR}/src/optim.cpp
${CMAKE_SOURCE_DIR}/src/ransac.cpp
)
    
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/../LBD_and_LineMatching/build/)
LINK_DIRECTORIES(${LAPACKBLAS_DIR})
# Link external libraries (opencv, eigen and our custom library)
target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${LAPACKBLAS_LIB_NAMES}
    # ${F2C_LIB_NAME}
    LineMatchingLib
    levmar
)

# Add main executable file
add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(main PUBLIC levmar ${PROJECT_NAME} )
# target_link_libraries(optim PUBLIC levmar ${PROJECT_NAME})

# Add unit_test executables
# add_executable(levmar_test ${CMAKE_CURRENT_SOURCE_DIR}/levmar_test.cpp)
target_link_libraries(main PUBLIC ${PROJECT_NAME} )

add_executable(test_ransac ${CMAKE_CURRENT_SOURCE_DIR}/test_ransac.cpp)
target_link_libraries(test_ransac PUBLIC ${PROJECT_NAME} )
