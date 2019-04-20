#pragma once

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "Matt6Estimator.hpp"
#include <vector>
#include <chrono>

using namespace Eigen;
using namespace std;

class EKF:public Matt6Estimator
{

public: //variables only

    //(pos)3+(vel)3+(quat)4+(feet_pos)18+(bias_acc)3+(bias_gyro)3;
    static const int state_dim = 34;
    //(feet_pos)
    static const int measure_dim = 18;
    //flag to check if there is a prior before we do prediction
    //we might be able to get rid of it in future
    bool prior_updated;

    typedef Matrix<double,state_dim,1> stateVecEKF;
    typedef Matrix<double,state_dim-1,state_dim-1> covMatrixEKF;
    
    typedef Matrix<double,state_dim-1,state_dim-1> QMatrix;
    typedef Matrix<double,state_dim-1,state_dim-1> FMatrix;
    typedef Matrix<double,measure_dim,measure_dim> RMatrix;
    typedef Matrix<double,measure_dim,state_dim-1> HMatrix;

public://functions only

    //constructor
    EKF();

    //to check if prior is initialized
    bool estimator_init();

    //state has to be in world frame
    //assuming the initial position of the robot to be the inertial/world frame
    bool updateCurrState(const std::vector<Matrix4d>& feet_pose_vec);
    
    //predict using IMU
    bool predict(const Vector3d& av_acc,const Vector3d& av_angvel);
    //update using encoders
    bool update(const std::vector<Matrix4d>& feet_pose_vec, const std::vector<Eigen::MatrixXd>& jac_vec);

    //calls both predict and update step
    void filtering(const Vector3d& av_acc, const Vector3d& av_angvel,
                        const std::vector<Matrix4d>& feet_pose_vec, const std::vector<Eigen::MatrixXd>& jac_vec );

private:
    //34x1 (3+3+4+(6x3)+3+3)
    stateVecEKF curr_state;
    //33x33
    covMatrixEKF curr_cov;
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

    struct noise_params
    {
        //3x3 diagonal matrix for easy calculation
        Matrix3d acc;
        //3x3 diagonal matrix for easy calculation
        Matrix3d bias_acc;
        //3x3 diagonal matrix for easy calculation
        Matrix3d gyro;
        //3x3 diagonal matrix for easy calculation
        Matrix3d bias_gyro;
        //3x3 diagonal matrix
        Matrix3d feet; 
        //3x3
        Matrix3d fk;
        //1x1
        double encoders; 
    }noise_params;
    

private:
    //hand-tune? //TO DO: set noises by collecting data, for now just hard set them
    void setSensorsNoise();
    void setProcessNoise(const Matrix3d& updated_rotm, const Vector3d& corrected_acc,const Vector3d& corrected_gyro);
    void setMeasureNoise(const std::vector<Eigen::MatrixXd>& jac_vec);

    //try finite difference method for which you will need process dynamics and measurement dynamics equations
    void setLinearizedProcessDynamics(const Matrix3d& updated_rotm, const Vector3d& corrected_acc, const Vector3d& corrected_gyro);
    void setLinearizedMeasureDynamics(const Matrix3d& pred_rotm, const VectorXd& pred_r, const VectorXd& pred_p);

    VectorXd getMeasurementResidual(const std::vector<Matrix4d>& feet_pose_vec, const Matrix3d& pred_rotm, 
                                            const VectorXd& pred_r, const VectorXd& pred_p);

};
