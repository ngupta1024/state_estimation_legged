#pragma once

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "matt6.hpp"
#include <vector>
using namespace Eigen;
using namespace std;
using namespace hebi;

class EKF
{
private://required for functions in public
    typedef Matrix<float,Matt6::state_dim-1,Matt6::state_dim-1> QMatrix;
    typedef Matrix<float,Matt6::state_dim-1,Matt6::state_dim-1> FMatrix;
    typedef Matrix<float,Matt6::measure_dim,Matt6::measure_dim> RMatrix;
    typedef Matrix<float,Matt6::measure_dim,Matt6::state_dim-1> HMatrix;

    QMatrix Q;
    RMatrix R;
    FMatrix F;
    HMatrix H;

public://functions only

    //constructor
    EKF();

    //to check if prior is initialized
    bool estimator_init();

    //hand-tune?
    void setProcessNoise(QMatrix& Q);
    void setMeasureNoise(RMatrix& R);

    //try finite difference method for which you will need process dynamics and measurement dynamics equations
    void setLinearizedProcessDynamics(Matt6::stateVecEKF* pred_state, FMatrix& F);
    void setLinearizedMeasureDynamics(HMatrix& H);
    
    //predict using IMU
    void predict();
    //update using encoders
    void update();

    //calls both predict and update step
    void filtering(Matt6 matt_obj);

public: //variables only
    bool bias_recorded;
    std::vector<Eigen::Vector3d> acc_bias_list;
    std::vector<Eigen::Vector3d> gyro_bias_list;

    Matt6::stateVecEKF curr_state;
    Matt6::covMatrixEKF curr_cov;

};
