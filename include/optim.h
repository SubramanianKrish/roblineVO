#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <Eigen/Core>

#include <iostream>
#include "levmar.h"

#include "frame.h"

#ifndef LM_DBL_PREC
#error Demo program assumes that levmar has been compiled with double precision, see LM_DBL_PREC!
#endif

#define ROSD 105.0

namespace optim
{

    class LineData{
    public:
        Eigen::Vector3d A, B, u, d;
        
        LineData(Eigen::Vector3d startPoint, Eigen::Vector3d endPoint): A(startPoint), B(endPoint){
            Eigen::Vector3d l = B - A;
            Eigen::Vector3d m = (A + B)/2;

            u = l/l.norm();
            d = (l.cross(m))/l.norm();
        }
    };

    struct RootInvCov{
        int idx1, idx2;
        std::vector<Eigen::Matrix3d> cov_matrices;
    };

    void compute_residual(double *points, double *error, int m, int n, void *data);

    points3d nonlinOptimize(points3d& line3D, std::vector<Eigen::Matrix3d>& inv_cov_one_line, std::vector<Eigen::Matrix3d>& covariance_matrices, std::vector<std::vector<Eigen::Matrix3d>>& endPt_covs, int line_idx1, int line_idx2);

    double m_dist(const Eigen::Vector3d& X, const Eigen::Matrix3d& sigma_x, const Eigen::Vector3d& A, const Eigen::Vector3d& B);

    double computeRtError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, 
                        const Eigen::Vector3d& A1, const Eigen::Vector3d& B1,
                        const Eigen::Matrix3d& cov_A1, const Eigen::Matrix3d& cov_B1,
                        const Eigen::Vector3d& A2, const Eigen::Vector3d& B2,
                        const Eigen::Matrix3d& cov_A2, const Eigen::Matrix3d& cov_B2
                        );


    // Get anti-symmetric matrix given a vector
    Eigen::Matrix3d GetCrossMatrix(const Eigen::Vector3d& vec);

    // Computing rotation matrix as per the reference paper
    Eigen::Matrix3d ComputeRotationMatrix(const LineData& l1, const LineData& l1_prime, const LineData& l2, const LineData& l2_prime);

    // Computing translation as per the reference paper
    Eigen::Vector3d ComputeTranslation(std::vector<LineData>& im1_lines, std::vector<LineData>& im2_lines, Eigen::Matrix3d& R);

    double computeRtError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& A1, 
                            const Eigen::Vector3d& B1, const Eigen::Vector3d& A2, const Eigen::Vector3d& B2);

    double computeDistError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& X, 
                            const Eigen::Vector3d& A, const Eigen::Vector3d& B);
}
