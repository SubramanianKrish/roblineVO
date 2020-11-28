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

    Ransac myRansac;
    double dist = myRansac.mahalanobis_distance(X, sigma, A, B);
}
