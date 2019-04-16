#pragma once

#include "est_utils.hpp"
#include <Eigen/Dense>
#include <vector>
// using namespace Eigen;
// using namespace std;
// using namespace hebi;

// base -> shoulder -> knee
struct Matt6FbkLeg 
{
    Eigen::Vector3d joint_ang;
    Eigen::Vector3d joint_vel;
    Eigen::Vector3d joint_tau;
};

// frame convention:  w: world frame, b: body frame, s: sensor frame
struct Matt6IMU 
{
    // acceleration in sensor frame
    Eigen::Vector3d acc_s;
    // acceleration in body frame     
    Eigen::Vector3d acc_b;  
    // angular velocity in sensor frame   
    Eigen::Vector3d gyro_s;
    // angular velocity in body frame
    Eigen::Vector3d gyro_b;
};

class Matt6
{
    //public variables
    public:
        //(pos)3+(vel)3+(quat)4+(feet_pos)18+(bias_acc)3+(bias_gyro)3;
        static const int state_dim=34;
        //(feet_pos)
        static const int measure_dim=18;
        //ekf specific
        typedef Matrix<double,state_dim-1,state_dim-1> covMatrixEKF;
        typedef Matrix<double,state_dim,1> stateVecEKF;
        typedef Matrix<double,measure_dim,1> measureStateEKF;

        std::vector<Matt6FbkLeg> fbk_legs;
        std::vector<Matt6IMU> fbk_imus;

    //public functions
    public:
        //constructor
        Matt6();

        // This function simulates the process dynamics discretely using fbk_imus
        // @params curr_state: const because we don't want it to change its contents
        // @return output_state:  
        void processDynamics(const Matt6::stateVecEKF& curr_state, Matt6::stateVecEKF& predicted_state);
        
        // This function simulates the measurement dyanmics using fbk_legs (joint angles, vel, effort)
        void measurementDynamics(const Matt6::stateVecEKF& predicted_state, Matt6::measureStateEKF& measurement_state);

};//class Matt6
