#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <Eigen/Core>

#include <iostream>
#include "levmar.h"

#ifndef LM_DBL_PREC
#error Demo program assumes that levmar has been compiled with double precision, see LM_DBL_PREC!
#endif

#define ROSD 105.0

/* Rosenbrock function, global minimum at (1, 1) */
void ros(double *p, double *x, int m, int n, void *data)
{
register int i;

  for(i=0; i<n; ++i)
    x[i]=((1.0-p[0])*(1.0-p[0]) + ROSD*(p[1]-p[0]*p[0])*(p[1]-p[0]*p[0]));
}

void jacros(double *p, double *jac, int m, int n, void *data)
{
register int i, j;

  for(i=j=0; i<n; ++i){
    jac[j++]=(-2 + 2*p[0]-4*ROSD*(p[1]-p[0]*p[0])*p[0]);
    jac[j++]=(2*ROSD*(p[1]-p[0]*p[0]));
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<double> lm_setup(const Eigen::MatrixXd& points){
    int n_points = points.rows();
    Eigen::Vector3d first_row = points.row(0);
    Eigen::Vector3d last_row = points.row(n_points-1);

    double line_length = (first_row - last_row).norm();    
    std::vector<double> param_vec;

    param_vec.push_back(points(0,0));
    param_vec.push_back(points(0,1));
    param_vec.push_back(points(0,2));
    for(int i = 1; i < n_points-1; i++)
    {
        Eigen::Vector3d row_i = points.row(i);
        double lambda = ((first_row - row_i).norm())/line_length;
        param_vec.push_back(lambda);
        // std::cout << lambda << std::endl;
    }
    param_vec.push_back(points(n_points-1,0));
    param_vec.push_back(points(n_points-1,1));
    param_vec.push_back(points(n_points-1,2));
    return param_vec;
}

void compute_residual(double *points, double *error, int m, int n, void *data){
    /*
    Function to compute residual after each iteration of levenberg-marquardt
    1. Additional data contains a pointer to the struct RootInvCov
    2. idx1 and idx2 refer to the beginning of the start point and the end point
    */
    // ToDo: Create a structure containing vector of inverse covariance matrices
    
    int para_idx = 0, error_idx = 0;
    struct RootInvCov* dptr;
    dptr = (struct RootInvCov *) adata;

    Eigen::Matrix3d startPoint = (Eigen::Matrix<double,3,1>()<< points[idx1], pts[idx1+1], pts[idx1+2]).finished();
    Eigen::Matrix3d endPoint = (Eigen::Matrix<double,3,1>()<< points[idx2], pts[idx2+1], pts[idx2+2]).finished();
    
    for(int i = 0; i < dptr.cov_matrices.size(); i++)
    {
        if(i == dptr->idx1)
        {
            Eigen::Matrix3d vec = dptr.cov_matrices(i) * startPoint;
            error[error_idx] = vec(0);
            error[error_idx + 1] = vec(1);
            error[error_idx + 2] = vec(2);
            para_idx = para_idx + 3;
        }
        else if(i == dptr->idx2)
        {
            Eigen::Matrix3d vec = dptr.cov_matrices(i) * endPoint;
            error[error_idx] = vec(0);
            error[error_idx + 1] = vec(1);
            error[error_idx + 2] = vec(2);
            para_idx = para_idx + 3;
        }
        else
        {
            Eigen::Matrix3d vec = dptr.cov_matrices(i) * (points[para_idx]*startPoint + (1-points[para_idx])*endPoint);
            error[error_idx] = vec(0);
            error[error_idx + 1] = vec(1);
            error[error_idx + 2] = vec(2);
            para_idx = para_idx + 1;
        }
        error_idx = error_idx + 3;
    }
    
}




// std::vector<double> lm_setup_meaurement_vec(const Eigen::MatrixXd& points, std::vector<double> params){
//     int n_points = points.rows();
    
// }


int main(int argc, char* argv[])
{ 
    register int i, j;
    int problem, ret;
    double p[5], // 5 is max(2, 3, 5)
        x[16]; // 16 is max(2, 3, 5, 6, 16)
    int m, n;
    double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    char *probname[]={
        "Rosenbrock function",
        "modified Rosenbrock problem",
        "Powell's function",
        "Wood's function",
        "Meyer's (reformulated) problem",
        "Osborne's problem",
        "helical valley function",
        "Boggs & Tolle's problem #3",
        "Hock - Schittkowski problem #28",
        "Hock - Schittkowski problem #48",
        "Hock - Schittkowski problem #51",
        "Hock - Schittkowski problem #01",
        "Hock - Schittkowski modified problem #21",
        "hatfldb problem",
        "hatfldc problem",
        "equilibrium combustion problem",
        "Hock - Schittkowski modified #1 problem #52",
        "Schittkowski modified problem #235",
        "Boggs & Tolle modified problem #7",
        "Hock - Schittkowski modified #2 problem #52",
        "Hock - Schittkowski modified problem #76",
    };

    opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
    opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing 
    // opts[4]=-LM_DIFF_DELTA; // specifies central differencing to approximate Jacobian; more accurate but more expensive to compute!

    problem = 0; // Meyer's (reformulated) problem

    m=2; n=2;
    p[0]=-1.2; p[1]=1.0;
    for(i=0; i<n; i++) x[i]=0.0;
    ret=dlevmar_der(ros, jacros, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic Jacobian
    //ret=dlevmar_dif(ros, p, x, m, n, 1000, opts, info, NULL, NULL, NULL);  // no Jacobian

    printf("Results for %s:\n", probname[problem]);
    printf("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);

    for(i=0; i<m; ++i)
        printf("%.7g ", p[i]);

    printf("\n\nMinimization info:\n");

    for(i=0; i<LM_INFO_SZ; ++i)
        printf("%g ", info[i]);

    printf("\n");

    std::cout << "Ho Gaya" << std::endl;


    Eigen::MatrixXd points = (Eigen::Matrix<double,3,3>()<< 
                             1.0, 3.1, 0.5,
                             0.4, 2.5, 1.0,
                             0.6, 3.2, 2.0).finished();

    std::vector<double> out = lm_setup(points);
    return 0;
}
