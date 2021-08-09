/**
 * \file calibMath.cpp
 * \brief Implementation of some math utility functions.
 * 
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#include <cmath>
#include "technology/mobilib/std/me_math.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibMatUtils.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibMath.h"
#include "technology/mobilib/float/common/MEmath/SVDmat.h"

namespace CalibUtils
{
mat33 fixrotm(mat33 const &M)
{
    Float::MEmath::SVDMat<3, 3, double> mat(M);
    mat33 out = mat.U() * mat.V().transpose();
    double d = Float::MEmath::determinant(out);
    if ( d < 0 )
    {
      mat33 J = Float::MEmath::identity<3, double>();
      J[2][2] = -1.0;
      out = mat.U() * J * mat.V().transpose();
    }
    return out;
}

double rotmdiff(mat33 const &R1, mat33 const &R2)
{
  mat33 R = R1.transpose() * R2;
  vec3 x = rotm2PitchYawRoll(R);
  return norm2(x)*DEGREES_PER_RADIAN;
}

double eps(double x)
{
    int exponent = -1074;
    if ( x != 0 )
    {
        exponent = -52.0 + std::floor(me_log2d(me_abs(x)));
    }
    return me_pow(2.0, exponent);
}

vec3 cross(vec3 const &a, vec3 const &b)
{
    vec3 c;
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
    return c;
}
} // namespace CalibUtils
