#pragma once

#include "Eigen/Eigen"

using namespace Eigen;

#define EPSILON 0.0001

typedef Matrix<int, 6, 1> Vector6i;
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<double, 6, 1> Vector6d;

// typedef Matrix<int, 6, 1> Vector6i;
// typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<double, 3, 1> Vector3d;

typedef Matrix<double, 7, 1> Vector7d;

typedef Matrix<double, 4, 1> Vector4d;

typedef Matrix<double, 6, 6> Matrix6d;
