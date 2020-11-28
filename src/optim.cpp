#include "optim.h"

namespace optim{

    void compute_residual(double *points, double *error, int m, int n, void *data){
        /*
        Function to compute residual after each iteration of levenberg-marquardt
        1. Additional data contains a pointer to the struct RootInvCov
        2. idx1 and idx2 refer to the beginning of the start point and the end point
        */
        // TODO: Create a structure containing vector of inverse covariance matrices
        
        int para_idx = 0, error_idx = 0;
        struct RootInvCov* dptr;

        dptr = (struct RootInvCov *) data;
        int idx1 = 0;
        int idx2 = dptr->idx2;
        
        Eigen::Vector3d startPoint = (Eigen::Matrix<double,3,1>()<< points[idx1], points[idx1+1], points[idx1+2]).finished();
        Eigen::Vector3d endPoint = (Eigen::Matrix<double,3,1>()<< points[idx2], points[idx2+1], points[idx2+2]).finished();
        std::vector<Eigen::Matrix3d> cov_mats = dptr->cov_matrices;

        for(int i = 0; i < (cov_mats).size(); i++)
        {
            if(i == idx1)
            {
                Eigen::Vector3d vec = cov_mats[i] * startPoint;
                error[error_idx] = vec(0);
                error[error_idx + 1] = vec(1);
                error[error_idx + 2] = vec(2);
                para_idx = para_idx + 3;
            }
            else if(i == idx2)
            {
                Eigen::Vector3d vec = cov_mats[i] * endPoint;
                error[error_idx] = vec(0);
                error[error_idx + 1] = vec(1);
                error[error_idx + 2] = vec(2);
                para_idx = para_idx + 3;
            }
            else
            {
                Eigen::Vector3d vec = cov_mats[i] * (points[para_idx]*startPoint + (1-points[para_idx])*endPoint);
                error[error_idx] = vec(0);
                error[error_idx + 1] = vec(1);
                error[error_idx + 2] = vec(2);
                para_idx = para_idx + 1;
            }
            error_idx = error_idx + 3;
        }

    }

    void nonlinOptimize(points3d& line3D, std::vector<Eigen::Matrix3d> inv_cov_one_line, int line_idx1, int line_idx2){
        std::cout << "Check pt 1" << std::endl;
        std::vector<double> param_vector;
        std::vector<double> ref_vector;
        Eigen::Vector3d startPoint = (Eigen::Matrix<double,3,1>()<< line3D(0,line_idx1), line3D(1,line_idx1), line3D(2, line_idx1)).finished();
        std::cout << "Check pt 2" << std::endl;
        Eigen::Vector3d endPoint = (Eigen::Matrix<double,3,1>()<< line3D(0, line_idx2), line3D(1, line_idx2), line3D(2, line_idx2)).finished();
        double lambda;
        double line_length = (startPoint-endPoint).norm();
        int n_points = inv_cov_one_line.size();
        param_vector.clear();
        std::cout << "Check pt 3" << std::endl;
        for (int j = 0; j < n_points; j++)
        {    
            if (j == line_idx1)
            {
                // std::cout << "IDX1" << std::endl;
                param_vector.push_back(line3D(0,j));
                param_vector.push_back(line3D(1,j));
                param_vector.push_back(line3D(2,j));
            }
            else if (j == line_idx2)
            {
                // std::cout << "IDX2" << std::endl;
                param_vector.push_back(line3D(0,j));
                param_vector.push_back(line3D(1,j));
                param_vector.push_back(line3D(2,j));
            }
            else
            {   Eigen::Vector3d intermediate_point = (Eigen::Matrix<double,3,1>()<< line3D(0, j), line3D(1, j), line3D(2, j)).finished();
                double dist = (startPoint - intermediate_point).norm();
                // std::cout << line_length << " " << dist << std::endl;
                double lambda = dist/line_length;
                // std::cout << lambda << std::endl;
                param_vector.push_back(lambda);
            }

            Eigen::Vector3d point = (Eigen::Matrix<double,3,1>()<< line3D(0, j), line3D(1, j), line3D(2, j)).finished();
            Eigen::Vector3d transformed_point = inv_cov_one_line[j]*point;
            ref_vector.push_back(transformed_point(0));
            ref_vector.push_back(transformed_point(1));
            ref_vector.push_back(transformed_point(2));
        }
        std::cout << "Check pt 4" << std::endl;
        int numPara = param_vector.size();

        double* para = new double[numPara];
        for (int i=0; i<numPara; ++i) {
            para[i] = param_vector[i];
            // std::cout << para[i] << std::endl;
        }
        int numRef = ref_vector.size();
        double* ref = new double[numRef];
        for ( int i=0; i<numRef; ++i) {
            ref[i] = ref_vector[i];
        }
        std::cout << "Check pt 5" << std::endl;
        optim::RootInvCov data;
        data.cov_matrices = inv_cov_one_line;
        data.idx1 = line_idx1;
        data.idx2 = line_idx2;
        double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
        opts[0] = LM_INIT_MU; //
        opts[1] = 1E-10; // gradient threshold, original 1e-15
        opts[2] = 1E-20; // relative para change threshold? original 1e-50
        opts[3] = 1E-20; // error threshold (below it, stop)
        opts[4] = LM_DIFF_DELTA;
        std::cout << "Check pt 6" << " " << para[5] << std::endl;
        int ret = dlevmar_dif(compute_residual, para, ref, numPara, numRef, 100, opts, info, NULL, NULL, (void*)&data);
        std::cout << "Check pt 7" << " " << para[5] << std::endl;
        delete[] para;
        delete[] ref;
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Rosenbrock function, global minimum at (1, 1) */
// void ros(double *p, double *x, int m, int n, void *data)
// {
// register int i;

//   for(i=0; i<n; ++i)
//     x[i]=((1.0-p[0])*(1.0-p[0]) + ROSD*(p[1]-p[0]*p[0])*(p[1]-p[0]*p[0]));
// }

// void jacros(double *p, double *jac, int m, int n, void *data)
// {
// register int i, j;

//   for(i=j=0; i<n; ++i){
//     jac[j++]=(-2 + 2*p[0]-4*ROSD*(p[1]-p[0]*p[0])*p[0]);
//     jac[j++]=(2*ROSD*(p[1]-p[0]*p[0]));
//   }
// }

// int main(int argc, char* argv[])
// { 
//     register int i, j;
//     int problem, ret;
//     double p[5], // 5 is max(2, 3, 5)
//         x[16]; // 16 is max(2, 3, 5, 6, 16)
//     int m, n;
//     double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
//     char *probname[]={
//         "Rosenbrock function",
//         "modified Rosenbrock problem",
//         "Powell's function",
//         "Wood's function",
//         "Meyer's (reformulated) problem",
//         "Osborne's problem",
//         "helical valley function",
//         "Boggs & Tolle's problem #3",
//         "Hock - Schittkowski problem #28",
//         "Hock - Schittkowski problem #48",
//         "Hock - Schittkowski problem #51",
//         "Hock - Schittkowski problem #01",
//         "Hock - Schittkowski modified problem #21",
//         "hatfldb problem",
//         "hatfldc problem",
//         "equilibrium combustion problem",
//         "Hock - Schittkowski modified #1 problem #52",
//         "Schittkowski modified problem #235",
//         "Boggs & Tolle modified problem #7",
//         "Hock - Schittkowski modified #2 problem #52",
//         "Hock - Schittkowski modified problem #76",
//     };

//     opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
//     opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing 
//     // opts[4]=-LM_DIFF_DELTA; // specifies central differencing to approximate Jacobian; more accurate but more expensive to compute!

//     problem = 0; // Meyer's (reformulated) problem

//     m=2; n=2;
//     p[0]=-1.2; p[1]=1.0;
//     for(i=0; i<n; i++) x[i]=0.0;
//     ret=dlevmar_der(ros, jacros, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic Jacobian
//     //ret=dlevmar_dif(ros, p, x, m, n, 1000, opts, info, NULL, NULL, NULL);  // no Jacobian

//     printf("Results for %s:\n", probname[problem]);
//     printf("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);

//     for(i=0; i<m; ++i)
//         printf("%.7g ", p[i]);

//     printf("\n\nMinimization info:\n");

//     for(i=0; i<LM_INFO_SZ; ++i)
//         printf("%g ", info[i]);

//     printf("\n");

//     std::cout << "Ho Gaya" << std::endl;


//     Eigen::MatrixXd points = (Eigen::Matrix<double,3,3>()<< 
//                              1.0, 3.1, 0.5,
//                              0.4, 2.5, 1.0,
//                              0.6, 3.2, 2.0).finished();

//     // std::vector<double> out = lm_setup(points);
//     return 0;
// }