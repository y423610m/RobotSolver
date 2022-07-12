#include "dh_parameters.h"

#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include <vector>
using namespace std;

namespace cvt{

    Matrix<double, 6, 1> toMat61XYZQuat(const vector<double>& targetPose);
    Matrix<double, 6, 1> toMat61XYZQuat(const Matrix4d& tipPose);

    Matrix<double, 6, 1> toMat61XYZEuler(const vector<double>& targetPose);
    Matrix<double, 6, 1> toMat61XYZEuler(const Matrix4d& tipPose);

    vector<double> toVecXYZEuler(const Matrix4d& Mat);

    Matrix4d toMat44RotZ(const double& angle);

    Matrix4d toMat44FromDH(const DHParam& dh);
    Matrix4d toMat44TRFromDH(const DHParam& dh, const double& jointAngle);
    Matrix4d toMat44RTFromDH(const double& jointAngle, const DHParam& dh);


}