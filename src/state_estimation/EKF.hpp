#pragma once

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "matt6.hpp"
#include <vector>
#include <chrono>

using namespace Eigen;
using namespace std;

class EKF
{

public: //variables only

    bool bias_recorded;
    std::vector<double> acc_bias_list;
    std::vector<double> gyro_bias_list;
    std::chrono::duration<double> fbk_dt;
    //flag to check if there is a prior before we do prediction
    //we might be able to get rid of it in future
    bool prior_updated;

    typedef Matrix<double,Matt6::state_dim-1,Matt6::state_dim-1> QMatrix;
    typedef Matrix<double,Matt6::state_dim-1,Matt6::state_dim-1> FMatrix;
    typedef Matrix<double,Matt6::measure_dim,Matt6::measure_dim> RMatrix;
    typedef Matrix<double,Matt6::measure_dim,Matt6::state_dim-1> HMatrix;

public://functions only

    //constructor
    EKF();

    //to check if prior is initialized
    bool estimator_init();

    //state has to be in world frame
    //assuming the initial position of the robot to be the inertial/world frame
    bool updateCurrState(std::vector<Matrix4d> feet_pose);

    //hand-tune?
    void setSensorsNoise();
    void setProcessNoise(Matrix3d updated_rotm, Vector3d corrected_acc, Vector3d corrected_gyro);
    void setMeasureNoise();

    //try finite difference method for which you will need process dynamics and measurement dynamics equations
    void setLinearizedProcessDynamics(Matrix3d updated_rotm, Vector3d corrected_acc, Vector3d corrected_gyro);
    void setLinearizedMeasureDynamics();
    
    //predict using IMU
    bool predict(Vector3d av_acc, Vector3d av_angvel, std::vector<Matt6IMU> imu_info);
    //update using encoders
    bool update();

    //calls both predict and update step
    void filtering(Matt6 matt_obj);

private:
    //34x1 (3+3+4+(6x3)+3+3)
    Matt6::stateVecEKF curr_state;
    //33x33
    Matt6::covMatrixEKF curr_cov;
    //33x33
    QMatrix Q;
    //18x18
    RMatrix R;
    //33x33
    FMatrix F;
    //18x33
    HMatrix H;
    //3x1
    Vector3d grav_vec;
    //1x1
    double dt;
    //3x3 diagonal matrix for easy calculation
    Matrix3d noise_acc;
    //3x3 diagonal matrix for easy calculation
    Matrix3d noise_bias_acc;
    //3x3 diagonal matrix for easy calculation
    Matrix3d noise_gyro;
    //3x3 diagonal matrix for easy calculation
    Matrix3d noise_bias_gyro;
    //3x3 diagonal matrix
    Matrix3d noise_feet;   

};
