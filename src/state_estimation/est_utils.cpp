#include "est_utils.hpp"


Quaterniond quatMult(Quaterniond q1, Quaterniond q2) {

    Quaterniond resultQ;
    resultQ.setIdentity();
    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}