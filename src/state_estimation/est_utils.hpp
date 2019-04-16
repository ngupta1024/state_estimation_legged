#pragma once

#include <Eigen/Dense>
using namespace Eigen;

namespace utils {

    Quaterniond quatMult(Quaterniond q1, Quaterniond q2);
    Matrix3d vec2skewsym(Vector3d x);
    Matrix3d quat2rotm(Quaterniond q);
    Matrix3d rodrigues(Vector3d axis_ang,int subscript);
    Matrix3d rodrigues(Vector3d axis, double ang, int subscript);
    Quaterniond axang2quat(Vector3d angle_axis);
}