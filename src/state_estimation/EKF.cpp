#include "EKF.hpp"
#include "est_utils.hpp"
#include <cmath>

using namespace Eigen;
using namespace std;

EKF::EKF()
{
    bias_recorded=false;
    acc_bias_list.resize(0);
    gyro_bias_list.resize(0);
    curr_state.fill(0);
    grav_vec.fill(0);
    grav_vec(2)=9.8;
}

// @return false if all zeros
bool EKF::estimator_init()
{
    return !curr_state.isZero(0);
}

//TO DO: in future use orientation of the feet too if that makes sense
bool EKF::updateCurrState(std::vector<Matrix4d> feet_pose)
{
    curr_state.fill(0);
    curr_state.segment<6>(0)<<0,0,0,0,0,0;
    curr_state.segment<4>(6)<<1,0,0,0;
    curr_state.segment<18>(10)<<feet_pose[0].topRightCorner(3,1),
                                feet_pose[1].topRightCorner(3,1),
                                feet_pose[2].topRightCorner(3,1),
                                feet_pose[3].topRightCorner(3,1),
                                feet_pose[4].topRightCorner(3,1),
                                feet_pose[5].topRightCorner(3,1);
                                
    curr_state.segment<3>(28)<<acc_bias_list[0],acc_bias_list[0],acc_bias_list[0];
    curr_state.segment<3>(31)<<gyro_bias_list[0],gyro_bias_list[0],gyro_bias_list[0];

    return true;
}

void EKF::setSensorsNoise()
{
    Matrix3d I;
    I.setIdentity(3,3);
    noise_acc=0.01*I;
    noise_bias_acc=0.001*I;
    noise_gyro=0.01*I;
    noise_bias_gyro=0.001*I;
    noise_feet=0.01*I;
}

void EKF::setLinearizedProcessDynamics(Matrix3d updated_rotm, Vector3d corrected_acc, Vector3d corrected_gyro)
{
    Matrix3d I;
    I.setIdentity(3,3);
    F.setIdentity(33,33);
    F.block<3,3>(0,3)=dt*I;
    F.block<3,3>(0,6)=-(pow(dt,2)/2)*(updated_rotm.transpose())*utils::vec2skewsym(corrected_acc);
    F.block<3,3>(0,27)=-(pow(dt,2)/2)*(updated_rotm.transpose());
    F.block<3,3>(3,6)=-dt*updated_rotm.transpose()*utils::vec2skewsym(corrected_acc);
    F.block<3,3>(3,27)=-dt*updated_rotm.transpose();
    F.block<3,3>(6,6)=utils::rodrigues(corrected_gyro,0).transpose();
    F.block<3,3>(6,30)=-1*utils::rodrigues(corrected_gyro,1).transpose();
}

void EKF::setLinearizedMeasureDynamics()
{
    H.fill(0);
}

void EKF::setProcessNoise(Matrix3d updated_rotm, Vector3d corrected_acc, Vector3d corrected_gyro)
{
    Q.fill(0);
    Q.block<3,3>(0,0)=(pow(dt,3)/3)*noise_acc+(pow(dt,5)/20)*noise_bias_acc;
    Q.block<3,3>(0,3)=(pow(dt,2)/2)*noise_acc+(pow(dt,4)/8)*noise_bias_acc;
    Q.block<3,3>(0,27)=-1*(pow(dt,3)/6)*updated_rotm.transpose()*noise_bias_acc;
    Q.block<3,3>(3,0)=(pow(dt,2)/2)*noise_acc+(pow(dt,4)/8)*noise_bias_acc;
    Q.block<3,3>(3,3)=dt*noise_acc+(pow(dt,3)/3)*noise_bias_acc;
    Q.block<3,3>(3,27)=-1*(pow(dt,2)/2)*updated_rotm.transpose()*noise_bias_acc;
    Q.block<3,3>(6,6)=dt*noise_gyro+(utils::rodrigues(corrected_gyro,3)+utils::rodrigues(corrected_gyro,3).transpose())*noise_bias_gyro;
    Q.block<3,3>(9,9)= dt*updated_rotm.transpose()*noise_feet*updated_rotm;
    Q.block<3,3>(12,12)=dt*updated_rotm.transpose()*noise_feet*updated_rotm;
    Q.block<3,3>(15,15)=dt*updated_rotm.transpose()*noise_feet*updated_rotm;
    Q.block<3,3>(18,18)=dt*updated_rotm.transpose()*noise_feet*updated_rotm;
    Q.block<3,3>(21,21)=dt*updated_rotm.transpose()*noise_feet*updated_rotm;
    Q.block<3,3>(24,24)=dt*updated_rotm.transpose()*noise_feet*updated_rotm;
    Q.block<3,3>(27,0)=-1*(pow(dt,3)/6)*noise_bias_acc*updated_rotm;
    Q.block<3,3>(27,3)=-1*(pow(dt,2)/2)*noise_bias_acc*updated_rotm;
    Q.block<3,3>(27,27)= dt*noise_bias_acc;
    Q.block<3,3>(30,6)=-1*noise_bias_gyro*(utils::rodrigues(corrected_gyro,2));
    Q.block<3,3>(30,30)= dt*noise_bias_gyro;   
}

void EKF::setMeasureNoise()
{

}

bool EKF::predict(Vector3d av_acc, Vector3d av_angvel, std::vector<Matt6IMU> imu_info)
{
    dt=fbk_dt.count();
    //to make code readable
    VectorXd updated_r=curr_state.segment<3>(0);
    VectorXd updated_v=curr_state.segment<3>(3);
    VectorXd updated_q_vec=curr_state.segment<4>(6);
    Quaterniond updated_q(updated_q_vec[0],updated_q_vec[1],updated_q_vec[2],updated_q_vec[3]);
    Matrix3d updated_rotm=utils::quat2rotm(updated_q);
    VectorXd updated_p=curr_state.segment<18>(10);
    Vector3d updated_bacc=curr_state.segment<3>(28);
    Vector3d updated_bgyro=curr_state.segment<3>(31);
    
    //equation 36 and 37
    Vector3d corrected_acc=av_acc-updated_bacc;
    Vector3d corrected_gyro=av_angvel-updated_bgyro;

    //equation 30-35
    curr_state.segment<3>(0)=updated_r+dt*(updated_v)+pow(dt,2)/2*(updated_rotm.transpose()*corrected_acc+grav_vec);
    curr_state.segment<3>(3)=updated_v+dt*(updated_rotm.transpose()*corrected_acc+grav_vec);
    Quaterniond predicted_q=utils::quatMult(utils::axang2quat(dt*corrected_gyro),updated_q);
    curr_state.segment<4>(6)<<predicted_q.w(),predicted_q.x(),predicted_q.y(),predicted_q.z();
    //rest remain same

    //set F
    setLinearizedProcessDynamics(updated_rotm,corrected_acc,corrected_gyro); 
    //set Q
    setProcessNoise(updated_rotm,corrected_acc,corrected_gyro);
    
    //set State Covariance
    curr_cov=((F*curr_cov)*F.transpose())+Q;

    return true;

}

// bool EKF::update()
// {
//     VectorXd y_residual=getMeasurementResidual();
//     //18x33
//     setLinearizedMeasureDynamics();
//     //18x18
//     setMeasureNoise();
//     //18x18
//     MatrixXd S=H*curr_cov*H.transpose()+R;
//     //33x18
//     MatrixXd K=curr_cov*H.transpose()*S.inverse();
//     //33x1
//     MatrixXd delta_x=K*y_residual;
//     curr_cov=(MatrixXd::Identity(33,33)-K*H)*curr_cov;

//     //add delta_x back to curr_state

// }