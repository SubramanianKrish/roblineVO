#include "optim.h"
#include <fstream>
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
        Eigen::Vector3d endPoint = (Eigen::Matrix<double,3,1>()<< points[idx2+2], points[idx2+3], points[idx2+4]).finished();
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
            else if(i == idx2 + 2)
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

    points3d nonlinOptimize(points3d& line3D, std::vector<Eigen::Matrix3d>& inv_cov_one_line, std::vector<Eigen::Matrix3d>& covariance_matrices, std::vector<std::vector<Eigen::Matrix3d>>& endPt_covs, int line_idx1, int line_idx2){
        // std::cout << "Check pt 1" << std::endl;
        std::vector<double> param_vector;
        std::vector<double> ref_vector;
        Eigen::Vector3d startPoint = line3D.col(line_idx1);
        // std::cout << "Check pt 2" << std::endl;
        Eigen::Vector3d endPoint = line3D.col(line_idx2);
        double lambda;
        double line_length = (startPoint-endPoint).norm();
        int n_points = inv_cov_one_line.size();
        param_vector.clear();
        // std::cout << "Check pt 3" << std::endl;
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
            {   Eigen::Vector3d intermediate_point = line3D.col(j);
                double dist = (startPoint - intermediate_point).norm();
                // std::cout << line_length << " " << dist << std::endl;
                double lambda = dist/line_length;
                // std::cout << lambda << std::endl;
                param_vector.push_back(lambda);
            }

            Eigen::Vector3d point = line3D.col(j);
            Eigen::Vector3d transformed_point = inv_cov_one_line[j]*point;
            ref_vector.push_back(transformed_point(0));
            ref_vector.push_back(transformed_point(1));
            ref_vector.push_back(transformed_point(2));
        }
        // std::cout << "Check pt 4" << std::endl;
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
        // std::cout << "Check pt 5" << std::endl;
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

        double *err = new double[numRef];
        // std::cout << "Check pt 6" << " " << para[5] << std::endl;
        // std::cout << "InvCovRoot inside optimizer = " << inv_cov_one_line[0] << std::endl;
        // std::cout << "InvCov inside optimizer = " << inv_cov_one_line[0]*inv_cov_one_line[0].transpose() << std::endl;
        // compute_residual(para, err, numPara, numRef, &data);
        // float sum = 0;
        // for (int i = 0; i < numRef; i++)
        // {
        //     sum = sum + (err[i]-ref[i])*(err[i]-ref[i]);
        // }
        // std::cout << " ------------------------------------------------ " << std::endl;
        // std::cout << "Sum before = " << sum << std::endl;
        int ret = dlevmar_dif(compute_residual, para, ref, numPara, numRef, 20, opts, info, NULL, NULL, (void*)&data);

        // compute_residual(para, err, numPara, numRef, &data);
        // sum = 0;
        // for (int i = 0; i < numRef; i++)
        // {
        //     sum = sum + (err[i]-ref[i])*(err[i]-ref[i]);
        // }
        // std::cout << "Sum After = " << sum << std::endl;


        // std::cout << "Check pt 7" << " " << para[5] << std::endl;
        // for (int i = 0; i < param_vector.size(); i++)
        // {
        //     std::cout << param_vector[i] << " " << para[i] << std::endl;
        // }
        points3d optimized_line(3, inv_cov_one_line.size());
        int para_idx = 0;
        Eigen::Vector3d sPoint = (Eigen::Matrix<double,3,1>()<< para[0], para[1], para[2]).finished();
        Eigen::Vector3d ePoint = (Eigen::Matrix<double,3,1>()<< para[numPara - 3], para[numPara - 2], para[numPara - 1]).finished();
        for (int i = 0; i < inv_cov_one_line.size(); i++)
        {
            if (i == line_idx1)
            {
                optimized_line(0,i) = para[para_idx];
                optimized_line(1,i) = para[para_idx + 1];
                optimized_line(2,i) = para[para_idx + 2];
                para_idx = para_idx + 3;
            }
            else if (i == line_idx2)
            {
                optimized_line(0,i) = para[para_idx];
                optimized_line(1,i) = para[para_idx + 1];
                optimized_line(2,i) = para[para_idx + 2];
                para_idx = para_idx + 3;
            }
            else
            {
                optimized_line(0,i) = para[para_idx]*sPoint(0) + (1 - para[para_idx])*ePoint(0);
                optimized_line(1,i) = para[para_idx]*sPoint(1) + (1 - para[para_idx])*ePoint(1);
                optimized_line(2,i) = para[para_idx]*sPoint(2) + (1 - para[para_idx])*ePoint(2);
                para_idx = para_idx + 1;
            }
        }

        Eigen::MatrixXd Jac = Eigen::MatrixXd::Zero(numPara, numRef);
        Jac(0,0) = 1;
        Jac(1,1) = 1;
        Jac(2,2) = 1;
        Jac(numPara-3,numRef-3) = 1;
        Jac(numPara-2,numRef-2) = 1;
        Jac(numPara-1,numRef-1) = 1;
        // Eigen::Vector3d row = Eigen::Matrix::Zero()
        // int idx_para = 3
        int id_1 = 0;
        int id_2 = numPara-3;
        int col = 0;
        for (int i = 0; i < numPara ; i++)
        {
            double parameter = para[i];
            if(i < id_1 + 3)
            {
                col = col + 1;
                continue;
            }
            else if (i >= id_2)
            {
                col = col + 1;
                continue;
            }
            else
            {
                Jac(0, col) = parameter;
                Jac(1, col + 1) = parameter;
                Jac(2, col + 2) = parameter;

                Jac(numPara - 3, col) = 1 - parameter;
                Jac(numPara - 2, col+1) = 1 - parameter;
                Jac(numPara - 1, col+2) = 1 - parameter;

                Jac(i, col) = para[0] - para[numPara - 3];
                Jac(i, col+1) = para[1] - para[numPara - 2];
                Jac(i, col+2) = para[2] - para[numPara - 1];

                col = col + 3;
            }
        }
        int inp;
        int size = covariance_matrices.size();
        Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(3*size, 3*size);
        // Eigen::MatrixXd temp2 = Eigen::MatrixXd::Zero(3*size, 3*size);
        for (int i = 0; i < size; i++)
        {
            temp.block(3*i, 3*i, 3, 3) = inv_cov_one_line[i].transpose()*inv_cov_one_line[i];
            // temp2.block(3*i, 3*i, 3, 3) = covariance_matrices[i];
            // if (i == 0)
            // {
            //     std::cout << numPara << std::endl;
            //     std::cout << "Inverse cov \n" << inv_cov_one_line[i].transpose()*inv_cov_one_line[i] << "\n" << "Cov matrix \n" << covariance_matrices[i] << std::endl;
                
            //     std::cout << Jac.bottomRightCorner(3,3) << std::endl;
            //     // for(long int k = 0; k < 100000000000; k++)
            //     //     continue;
            // }
        }
        // std::ofstream file("test.txt");
        // if (file.is_open())
        // {
        //     // MatrixXf m = MatrixXf::Random(30,3);
        //     file << "Here is the matrix m:\n" << temp*temp2 << '\n';
        //     // file << "m" << '\n' <<  colm(m) << '\n';
        // }
        // file.close();
        // std::cin >> inp;

        Eigen::MatrixXd estimated_cov = (Jac*temp*Jac.transpose()).inverse();
        
        std::vector<Eigen::Matrix3d> temp_vector;
        temp_vector.push_back(estimated_cov.topLeftCorner(3,3));
        temp_vector.push_back(estimated_cov.bottomRightCorner(3,3));

        endPt_covs.push_back(temp_vector);
        temp_vector.clear();
        // std::cout << Jac << std::endl;
        delete[] para;
        delete[] ref;
        return optimized_line;
    
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
        // std::cout << r[0]*r[0] << " " << r[1]*r[1] << " " << r[2]*r[2] << " " << r[3]*r[3] << "\n";
        return r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3];
    }
    
    double computeDistError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& X, 
                            const Eigen::Vector3d& A, const Eigen::Vector3d& B)
    {
        Eigen::Vector3d transformed_X = R*X + t;
        Eigen::Vector3d direction = B-A;
        double dist = ((transformed_X - A).cross(direction)).norm()/direction.norm();
        return dist;
    }

    double computeRtError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& A1, 
                            const Eigen::Vector3d& B1, const Eigen::Vector3d& A2, const Eigen::Vector3d& B2)
    {
        double err1 = computeDistError(R, t, A1, A2, B2);
        double err2 = computeDistError(R, t, B1, A2, B2);

        return (err1 + err2)*0.5;
    }

    Eigen::Matrix3d GetCrossMatrix(const Eigen::Vector3d& vec){
        Eigen::Matrix3d vec_hat = (Eigen::Matrix<double,3,3>()<<  0, -vec(2), vec(1),
                                                                vec(2), 0, -vec(0),
                                                                -vec(1), vec(0), 0).finished();
        return vec_hat;
    }

    Eigen::Matrix3d ComputeRotationMatrix(const LineData& l1, const LineData& l1_prime, const LineData& l2, const LineData& l2_prime){
        Eigen::Matrix4d A1, A2;

        // Constructing the matrix as per reference paper
        A1(0,0) = 0;
        A1.block(1,1,3,3) = optim::GetCrossMatrix(l1.u + l1_prime.u);
        A1.bottomLeftCorner(3,1) = -(l1.u - l1_prime.u);
        A1.topRightCorner(1,3) = (l1.u - l1_prime.u).transpose();

        // std::cout << A1 << "\n" << std::endl;

        A2(0,0) = 0;
        A2.block(1, 1, 3, 3) = optim::GetCrossMatrix(l2.u + l2_prime.u);
        A2.bottomLeftCorner(3,1) = -(l2.u - l2_prime.u);
        A2.topRightCorner(1,3) = (l2.u - l2_prime.u).transpose();

        // std::cout << A1.transpose()*A1 + A2.transpose()*A2 << "\n" << "\n";
        // Computing the SVD of the sum of the matrices
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A1.transpose()*A1 + A2.transpose()*A2, Eigen::ComputeFullU | Eigen::ComputeFullV);  
        Eigen::Matrix4d U = svd.matrixU();
        Eigen::Matrix4d V = svd.matrixV();

        // std::cout << "Matrix Product \n" << U << "\n" << V << std::endl;
        // int a;
        // std::cin >> a;
        Eigen::Vector4d temp_Q = V.col(3);

        // Initialize a quaternion (not sure if the returned vector is (w, x, y, z), or (x, y, z, w))
        Eigen::Quaterniond q(temp_Q(0), temp_Q(1), temp_Q(2), temp_Q(3));

        // Converting quaternion to rotation matrix
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();

        // std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        // std::cout << R << "\n" << std::endl;

        return R;

    }


    Eigen::Vector3d ComputeTranslation(std::vector<LineData>& im1_lines, std::vector<LineData>& im2_lines, Eigen::Matrix3d& R){
        Eigen::Matrix3d temp1 = Eigen::Matrix3d::Zero(3,3);
        Eigen::Vector3d temp2 = Eigen::Vector3d::Zero(3);
        for (int i = 0; i < im2_lines.size(); i++)
        {
            Eigen::Matrix3d u_prime_hat = optim::GetCrossMatrix(im2_lines[i].u);
            temp1 = temp1 + u_prime_hat*u_prime_hat.transpose();

            temp2 = temp2 + u_prime_hat.transpose()*(im2_lines[i].d - R*im1_lines[i].d);
        
        }
        Eigen::Vector3d t = temp1.inverse()*temp2;

        return t;
    }
}
