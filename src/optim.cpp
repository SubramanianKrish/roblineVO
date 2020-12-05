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

    double m_dist(const Eigen::Vector3d& X, const Eigen::Matrix3d& sigma_x, const Eigen::Vector3d& A, const Eigen::Vector3d& B)
    {   // <To-do> remove redundant code. Same already exists in ransac. Need to make a common method
    
        // Generate Eigen values of cov for ransac later
        Eigen::SelfAdjointEigenSolver<covariance> eigensolver(sigma_x);
        if (eigensolver.info() != Eigen::Success){
            cout << "Could not perform eigen decomposition on 3D point's covariance matrix" << endl;
            cout << sigma_x << endl;
            abort();
        }
        
        Eigen::Matrix3d U = eigensolver.eigenvectors();
        Eigen::Vector3d D = eigensolver.eigenvalues();

        // Apply affine transform to points
        Eigen::Vector3d A_affine = ((D.array().inverse()).sqrt()).matrix().asDiagonal() * U.transpose() * (A - X);
        Eigen::Vector3d B_affine = ((D.array().inverse()).sqrt()).matrix().asDiagonal() * U.transpose() * (B - X);

        double distance = A_affine.cross(B_affine).norm()/(A_affine - B_affine).norm();

        return distance;
    }

    double computeRtError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, 
                          const Eigen::Vector3d& A1, const Eigen::Vector3d& B1,
                          const Eigen::Matrix3d& cov_A1, const Eigen::Matrix3d& cov_B1,
                          const Eigen::Vector3d& A2, const Eigen::Vector3d& B2,
                          const Eigen::Matrix3d& cov_A2, const Eigen::Matrix3d& cov_B2
                          )
    {
        double r[4];
        r[0] = m_dist(R*A1 + t, R*cov_A1*(R.transpose()), A2, B2);
        r[1] = m_dist(R.transpose()*A2 - R.transpose()*t, R.transpose()*cov_A2*R, A1, B1);
        r[2] = m_dist(R*B1+t, R*cov_B1*(R.transpose()), A2, B2);
        r[3] = m_dist(R.transpose()*B2 - R.transpose()*t, R.transpose()*cov_B2*R, A1, B1);

        return r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3];
    }
    
}
