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

struct RootInvCov{
    int idx1, idx2;
    std::vector<Eigen::Matrix3d> cov_matrices;
};

void compute_residual(double *points, double *error, int m, int n, void *data);

points3d nonlinOptimize(points3d& line3D, std::vector<Eigen::Matrix3d> inv_cov_one_line, int line_idx1, int line_idx2);

}