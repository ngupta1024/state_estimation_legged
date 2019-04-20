#pragma once

#include "est_utils.hpp"
#include <Eigen/Dense>
#include <vector>
#include <chrono>

class Matt6Estimator
{
    //public variables
  public:
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

    bool bias_recorded;
    std::vector<double> acc_bias_list;
    std::vector<double> gyro_bias_list;
    std::chrono::duration<double> fbk_dt;

    std::vector<Matt6FbkLeg> fbk_legs;
    std::vector<Matt6IMU> fbk_imus;

    //public functions
  public:
    //constructor
    Matt6Estimator();

    // This function simulates the process dynamics discretely using fbk_imus
    // @params curr_state: const because we don't want it to change its contents
    // @return output_state:
    void processDynamics(const VectorXd& curr_state, VectorXd& predicted_state);

    // This function simulates the measurement dyanmics using fbk_legs (joint angles, vel, effort)
    void measurementDynamics(const VectorXd& predicted_state, VectorXd &measurement_state);

    //filtering using IMU and forward kinematics
    virtual void filtering(const Vector3d& av_acc, const Vector3d& av_angvel,
                        const std::vector<Matrix4d>& feet_pose_vec, const std::vector<Eigen::MatrixXd>& jac_vec)=0;

}; //class Matt6Estimator
