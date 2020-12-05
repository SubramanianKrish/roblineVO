#include <Eigen/Dense>
#include <iostream>

#include "ransac.h"

using namespace Eigen;
using namespace std;

int main(){
    Matrix3d sigma;
    sigma << 0.2, 0, 0, 0, 0.3, 0, 0, 0, 0.5;
    Vector3d X = Vector3d::Random(3,1);
    Vector3d A = Vector3d::Random(3,1);
    Vector3d B = Vector3d::Random(3,1);

    cout << "================" << endl;

    Ransac myRansac(100, 1e-4);
    double dist = myRansac.mahalanobis_distance(X, sigma, A, B);
    Matrix3Xd temp = MatrixXd::Random(3,100);
    vector<Matrix3d> cov;
    for(int i=0; i< 100; ++i){
        cov.push_back(sigma);
    }
    Eigen::MatrixXd temp2 = myRansac.removeOutlierPoints(temp, cov);

    return 0;
}
