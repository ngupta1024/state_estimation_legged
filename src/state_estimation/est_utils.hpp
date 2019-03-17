#pragma once

#include <Eigen/Dense>
using namespace Eigen;

namespace hebi {

    Quaterniond quatMult(Quaterniond q1, Quaterniond q2);

}