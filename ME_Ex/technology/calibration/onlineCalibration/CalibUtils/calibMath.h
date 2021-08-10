/**
 * \file calibMath.h
 * \brief Mathematical utility functions
 *
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#pragma once
#include <string>
#include <algorithm>
#include "technology/mobilib/std/me_math.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "technology/mobilib/float/common/MEmath/vec.h"
#include "technology/mobilib/float/common/MEmath/SVDmat.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"
#include "technology/mobilib/std/math/sqrt.h"

namespace me = Float::MEmath;
namespace CalibUtils
{
/////////////////////////////
// Non-templated functions //
/////////////////////////////

/**
 * Minimally pertub matrix so as to make it a rotation matrix, using SVD.
 */
mat33 fixrotm(mat33 const &A);

/**
 * Compute the angle in degrees of A'*B
 */
double rotmdiff(mat33 const &A, mat33 const &B);

/**
 * Compute cross-product between two vectors of length 3
 */
vec3 cross(vec3 const &a, vec3 const &b);

/**
 * Compute spacing to the next larger number in floating point. Like matlab's eps(x)
 * returns the distance from abs(x) to the next larger in magnitude double.
 */
double eps(double x = 1.0);

/////////////////////////
// Templated functions //
/////////////////////////

/**
 * Compute median of a vector of odd length (otherwise it is not clear what type to return)
 */
template <int LENGTH, class T>
T odd_length_median(me::Vec<LENGTH, T> v);

/**
 * Pseudo inverse. If tol is omitted, or negative, tolerance is computed as in matlab,
 * i.e., TOL = max(size(A)) * eps(norm(A)).
 */
template <int ROWS, int COLS>
me::Mat<COLS, ROWS> pinv(me::Mat<COLS, ROWS> const &A, double tol = -1.0);

/**
 * Linear least-squares solve, using pseudo-inverse with standard tolerance.
 * \param matrix A
 * \param right-hand-side vector b
 * \return pinv(A)*b
 */
template <int ROWS, int COLS, class T>
me::Vec<COLS, T> linsolve(me::Mat<COLS, ROWS, T> const &A, me::Vec<ROWS, T> const &b);

/**
 * Find max element of a vector.
 */
template <int ROWS, class T>
T max(me::Vec<ROWS, T> const &v);

/**
 * Compute 2-norm of a vector.
 */
template <int ROWS, class T>
double norm2(me::Vec<ROWS, T> const &v);

/**
 * Compute the Root-Mean-Square value of a vector.
 */
template <int ROWS, class T>
double rms(me::Vec<ROWS, T> const &v);

/**
 * Compute dot product
 * \param u vector
 * \param v vector
 * \return u.transpose() * v
 */
template <int ROWS, class T>
double dot(me::Vec<ROWS, T> const &u, me::Vec<ROWS, T> const &v);

/**
 * Compute exterior product.
 * \param u vector
 * \param v vector
 * \return u*v.transpose()
 */
template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> exteriorProd(me::Vec<ROWS, T> const &u, me::Vec<COLS, T> const &v);

/**
 * Compute Kronecker product
 * \param A matrix
 * \param B matrix
 * \return kron(A,B) (see https://en.wikipedia.org/wiki/Kronecker_product)
 */
template <int ROWS_A, int COLS_A, int ROWS_B, int COLS_B, class T>
me::Mat<ROWS_A * ROWS_B, COLS_A * COLS_B, T> kron(me::Mat<ROWS_A, COLS_A, T> const &A, me::Mat<ROWS_B, COLS_B, T> const &B);

/**
 * Compute Kronecker product
 * \param a vector
 * \param b vector
 * \return kron(a,b) (see https://en.wikipedia.org/wiki/Kronecker_product)
 */
template <int ROWS_A, int ROWS_B, class T>
me::Vec<ROWS_A * ROWS_B, T> kron(me::Vec<ROWS_A, T> const &a, me::Vec<ROWS_B, T> const &b);

/**
 * Compute Kronecker product
 * \param a vector
 * \param B matrix
 * \return kron(a,B) (see https://en.wikipedia.org/wiki/Kronecker_product)
 */
template <int ROWS_A, int ROWS_B, int COLS_B, class T>
me::Mat<ROWS_A * ROWS_B, COLS_B, T> kron(me::Vec<ROWS_A, T> const &a, me::Mat<ROWS_B, COLS_B, T> const &B);

/**
 * Compute Kronecker product
 * \param A matrix
 * \param b vector
 * \return kron(A,b) (see https://en.wikipedia.org/wiki/Kronecker_product)
 */
template <int ROWS_A, int COLS_A, int ROWS_B, class T>
me::Mat<ROWS_A * ROWS_B, COLS_A, T> kron(me::Mat<ROWS_A, COLS_A, T> const &A, me::Vec<ROWS_B, T> const &b);

/**
 * Compute matrix trace (sum of diagonal elements).
 */
template <int ROWS, int COLS, class T>
T trace(me::Mat<ROWS, COLS, T> const &A);

////////////////////////////////////////////
// Implementations of templated functions //
////////////////////////////////////////////
template <int ROWS, int COLS>
me::Mat<COLS, ROWS> pinv(me::Mat<COLS, ROWS> const &A, double tol)
{
    me::SVDMat<ROWS, COLS, double> mat(A);
    if (tol < 0)
    {
        tol = std::max(ROWS, COLS) * eps(CalibUtils::max(mat.W()));
    }
    return mat.pseudoInverse(tol);
}

template <int ROWS, class T>
T max(me::Vec<ROWS, T> const &v)
{
    return *(std::max_element(v.begin(), v.end()));
}

template <int ROWS, int COLS, class T>
me::Vec<COLS, T> linsolve(me::Mat<COLS, ROWS, T> const &A, me::Vec<ROWS, T> const &b)
{
    return pinv(A) * b;
}

template <int ROWS, class T>
double norm2(me::Vec<ROWS, T> const &v)
{
    return me_sqrtd(dot(v, v));
}

template <int ROWS, class T>
double rms(me::Vec<ROWS, T> const &v)
{
    return me_sqrtd(dot(v, v) / ROWS);
}

template <int ROWS, class T>
double dot(me::Vec<ROWS, T> const &u, me::Vec<ROWS, T> const &v)
{
    double out = 0.0;
    for (unsigned int i = 0; i < ROWS; i++)
    {
        out += u[i] * v[i];
    }
    return out;
}

template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> exteriorProd(me::Vec<ROWS, T> const &u, me::Vec<COLS, T> const &v)
{
    me::Mat<ROWS, COLS, T> out;
    for (unsigned int i = 0; i < ROWS; i++)
    {
        for (unsigned int j = 0; j < COLS; j++)
        {
            out(i, j) = u[i] * v[j];
        }
    }
    return out;
}

template <int ROWS_A, int COLS_A, int ROWS_B, int COLS_B, class T>
me::Mat<ROWS_A * ROWS_B, COLS_A * COLS_B, T> kron(me::Mat<ROWS_A, COLS_A, T> const &A, me::Mat<ROWS_B, COLS_B, T> const &B)
{
    me::Mat<ROWS_A * ROWS_B, COLS_A * COLS_B, T> out;

    for (unsigned int iA = 0; iA < ROWS_A; iA++)
    {
        for (unsigned int iB = 0; iB < ROWS_B; iB++)
        {
            for (unsigned int jA = 0; jA < COLS_A; jA++)
            {
                for (unsigned int jB = 0; jB < COLS_B; jB++)
                {
                    out(ROWS_B * iA + iB, COLS_B * jA + jB) = A(iA, jA) * B(iB, jB);
                }
            }
        }
    }
    return out;
}

template <int ROWS_A, int ROWS_B, class T>
me::Vec<ROWS_A * ROWS_B, T> kron(me::Vec<ROWS_A, T> const &a, me::Vec<ROWS_B, T> const &b)
{
    return vec(exteriorProd(b, a));
}

template <int ROWS_A, int ROWS_B, int COLS_B, class T>
me::Mat<ROWS_A * ROWS_B, COLS_B, T> kron(me::Vec<ROWS_A, T> const &a, me::Mat<ROWS_B, COLS_B, T> const &B)
{
    return kron(me::Mat<ROWS_A, 1, T>(a), B);
}

template <int ROWS_A, int COLS_A, int ROWS_B, class T>
me::Mat<ROWS_A * ROWS_B, COLS_A, T> kron(me::Mat<ROWS_A, COLS_A, T> const &A, me::Vec<ROWS_B, T> const &b)
{
    return kron(A, me::Mat<ROWS_B, 1, T>(b));
}

template <int ROWS, int COLS, class T>
T trace(me::Mat<ROWS, COLS, T> const &A)
{
    T out = 0;
    for (unsigned int i = 0; i < ROWS; i++)
    {
        out += A(i, i);
    }
    return out;
}

template <int LENGTH, class T>
T odd_length_median(me::Vec<LENGTH, T> v)
{
  assert(v.size() % 2 == 1);
  size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  T vn = v[n];  
  return vn;  
}

} // namespace CalibUtils