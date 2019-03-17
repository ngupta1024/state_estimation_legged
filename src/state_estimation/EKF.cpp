#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "EKF.hpp"

using namespace Eigen;
using namespace std;

EKF::EKF()
{
    bias_recorded=false;
    acc_bias_list.resize(0);
    gyro_bias_list.resize(0);
    curr_state.fill(0);
}

// @return false if all zeros
bool EKF::estimator_init()
{
    return !this->curr_state.isZero(0);
}