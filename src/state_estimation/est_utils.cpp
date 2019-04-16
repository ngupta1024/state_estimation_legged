#include "est_utils.hpp"
#include <cmath>

// using namespace utils;

Quaterniond utils::quatMult(Quaterniond q1, Quaterniond q2) 
{
    Quaterniond resultQ;
    resultQ.setIdentity();
    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}

Matrix3d utils::vec2skewsym(Vector3d x)
{
    Matrix3d x_cap;
    x_cap<<0,-x(2),x(1),
            x(2),0,-x(0),
            -x(1),x(0),0;
    return x_cap;
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
Matrix3d utils::quat2rotm(Quaterniond q)
{
    double sqw=q.w()*q.w();
    double sqx=q.x()*q.x();
    double sqy=q.y()*q.y();
    double sqz=q.z()*q.z();

    double invs=1/(sqw+sqx+sqy+sqz);
    double m00=( sqx - sqy - sqz + sqw)*invs ;
    double m11=(-sqx + sqy - sqz + sqw)*invs ;
    double m22 = (-sqx - sqy + sqz + sqw)*invs ;

    double tmp1 = q.x()*q.y();
    double tmp2 = q.z()*q.w();
    double m10 = 2.0 * (tmp1 + tmp2)*invs ;
    double m01 = 2.0 * (tmp1 - tmp2)*invs ;
    
    tmp1 = q.x()*q.z();
    tmp2 = q.y()*q.w();
    double m20 = 2.0 * (tmp1 - tmp2)*invs ;
    double m02 = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = q.y()*q.z();
    tmp2 = q.x()*q.w();
    double m21 = 2.0 * (tmp1 + tmp2)*invs ;
    double m12 = 2.0 * (tmp1 - tmp2)*invs ; 

    Matrix3d rotm;
    rotm<<m00,m01,m02,
                    m10,m11,m12,
                    m20,m21,m22;
    
    return rotm;
}

Matrix3d utils::rodrigues(Vector3d axis_ang,int subscript)
{
    Matrix3d rotm;
    double ang=axis_ang.norm();
    Vector3d axis=axis_ang/ang;
    return rodrigues(axis,ang,subscript);
}

Matrix3d utils::rodrigues(Vector3d axis, double ang, int subscript)
{
    Matrix3d rotm;
    rotm.setIdentity();
    Matrix3d w_cap=vec2skewsym(axis);
    switch(subscript)
    {
        case 0: 
            rotm = rotm+sin(ang)*w_cap+(1-cos(ang))*(w_cap*w_cap);
            break;

        case 1:
            rotm = rotm+(1-cos(ang))*(w_cap/ang)+(ang-sin(ang))*(w_cap*w_cap)/ang;
            break;
        
        case 2:
            rotm = rotm+(ang-sin(ang))*(w_cap/(ang*ang))+(((cos(ang)-1)+(pow(ang,2)/2))/(ang*ang))*(w_cap*w_cap);
            break;

        case 3:
            rotm = rotm+(cos(ang)-1+(pow(ang,2)/2))/(pow(ang,3))*w_cap+((sin(ang)-ang+(pow(ang,3)/6))/(pow(ang,3))*(w_cap*w_cap));
            break;
        
        default:
            break;

    }
    return rotm;
}

Quaterniond utils::axang2quat(Vector3d angle_axis)
{

}