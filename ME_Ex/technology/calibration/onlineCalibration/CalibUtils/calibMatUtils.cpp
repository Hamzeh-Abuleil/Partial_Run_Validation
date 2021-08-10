/**
 * \file calibMatUtils.cpp
 * \brief Implementation of some matrix utility functions.
 * 
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#include "technology/calibration/onlineCalibration/CalibUtils/calibMatUtils.h"
#include "technology/mobilib/float/common/MEmath/tMat.h"

namespace CalibUtils
{
mat33 tform2rotm(mat44 const &M)
{
    mat33 out;
    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned j = 0; j < 3; j++)
        {
            out(i, j) = M(i, j);
        }
    }
    return out;
}

mat33f tform2rotm(mat44f const &M)
{
    mat33f out;
    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned j = 0; j < 3; j++)
        {
            out(i, j) = M(i, j);
        }
    }
    return out;
}

vec3 tform2trvec(mat44 const &M)
{
    vec3 out;
    for (unsigned i = 0; i < 3; i++)
    {
        out[i] = M(i, 3);
    }
    return out;
}

vec3f tform2trvec(mat44f const &M)
{
    vec3f out;
    for (unsigned i = 0; i < 3; i++)
    {
        out[i] = M(i, 3);
    }
    return out;
}

vec3 tform2PitchYawRoll(mat44 const &M)
{
    return rotm2PitchYawRoll(tform2rotm(M));    
}

vec3 rotm2PitchYawRoll(mat33 const &R)
{
    vec3 out;
    Float::MEmath::R2ypr(R, out[1], out[0], out[2]);
    out[0] *= -1;
    return out;
}

mat44 tformFromRAndT(mat33 const& R, vec3 const& t)
{
    mat44 out;
    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            out(i, j) = R(i, j);
        }
        assign<12, 14>(out, t);
    }
    out(3, 3) = 1;
    return out;
}
} // namespace CalibUtils