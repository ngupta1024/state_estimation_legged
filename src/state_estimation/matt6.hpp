#pragma once

#include "est_utils.hpp"
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;
using namespace hebi;

namespace hebi{

struct Matt6FbkLeg 
{
    // base -> shoulder -> knee
    Vector3d joint_ang;
    Vector3d joint_vel;
    Vector3d joint_tau;
};

// frame convention:  w: world frame, b: body frame, s: sensor frame
struct Matt6IMU 
{
    Vector3d acc_s;   // in sensor frame 
    Vector3d acc_b;     // in body frame 
    Vector3d gyro_s;
    Vector3d gyro_b;
};

class Matt6
{
    public:

        static const int state_dim=34;//(pos)3+(vel)3+(quat)4+(feet_pos)18+(bias_acc)3+(bias_gyro)3;
        static const int measure_dim=18;//(feet_pos)
        //ekf specific
        typedef Matrix<float,state_dim,state_dim> covMatrixEKF;
        typedef Matrix<float,state_dim,1> stateVecEKF;
        typedef Matrix<float,measure_dim,1> measureStateEKF;

        std::vector<Matt6FbkLeg> fbk_legs;
        std::vector<Matt6IMU> fbk_imus;
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

}//namespace hebi
