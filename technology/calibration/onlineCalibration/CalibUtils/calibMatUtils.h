/**
 * \file calibMatUtils.h
 * \brief Interface for matrix utility functions.
 *
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#pragma once

#include "technology/mobilib/float/common/MEmath/mat.h"
#include <string>
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"

namespace me = Float::MEmath;
namespace CalibUtils
{
/**
 * Print a matrix, in a format ready for cutting and pasting into matlab.
 * \param in matrix to print.
 */
template <int ROWS, int COLS, class T>
void printMat(me::Mat<ROWS, COLS, T> const &in, std::string const &header = "");

/**
 * Print a vector, in a format ready for cutting and pasting into matlab.
 * \param in vector to print.
 */
template <int ROWS, class T>
void printVec(me::Vec<ROWS, T> const &in, std::string const &header = "");

/**
 * Extract the rotation sub-matrix from a 4x4 tform matrix.
 * \param M a tform matrix of the form [R t; 0 1]
 */
mat33 tform2rotm(mat44 const &M);

/**
 * Extract the translation vector from a 4x4 tform matrix.
 * \param M a tform matrix of the form [R t; 0 1]
 */
vec3 tform2trvec(mat44 const &M);


/**
 * Compute the Pitch, Yaw, and Roll, from a 4x4 tform matrix.
 * \param M a tform matrix of the form [R t; 0 1]
 */
vec3 tform2PitchYawRoll(mat44 const &M);

/**
 * Extract the rotation sub-matrix from a 4x4 tform matrix.
 * \param M a tform matrix of the form [R t; 0 1]
 */
mat33f tform2rotm(mat44f const &M);

/**
 * Extract the translation vector from a 4x4 tform matrix.
 * \param M a tform matrix of the form [R t; 0 1]
 */
vec3f tform2trvec(mat44f const &M);

/**
 * Compute the Pitch, Yaw, and Roll, from a 3x3 rotation matrix.
 * \param M a rotation matrix 
 */
vec3 rotm2PitchYawRoll(mat33 const &R);

/**
 * Form a tform (4x4 matrix) from R and t
 * \param R Rotation matrix
 * \param t translation vector
 * \return tform [R, t; 0 1]
 */
mat44 tformFromRAndT(mat33 const &R, vec3 const &t);

/**
 * Column-stack a matrix to form a vector. Like matlab's v = A(:).
 * \param A a matrix
 * \return v column-stacked version of A
 */
template <int ROWS, int COLS, class T>
me::Vec<ROWS * COLS, T> vec(me::Mat<ROWS, COLS, T> const &A);

/**
 * Generate a matrix from a vector in column-major order. Like matlab's
 * A = reshape(ROWS, COLS, v)
 * \param v input vector
 * \tparam ROWS
 * \tparam COLS
 * \return A matrix of size ROWS x COLS
 */
template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> mat(const me::Vec<ROWS * COLS, T> &v);

/**
 * Generate a matrix from an array in column-major order. Like matlab's
 * A = reshape(ROWS, COLS, v)
 * \param v input array
 * \tparam ROWS
 * \tparam COLS
 * \return A matrix of size ROWS x COLS
 */
template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> mat(const T* v);

/**
 * Extract a range of indices from a vector. So the call,
 * out = subvec<FIRST, LAST>(in) is analogous to out = in(start:end) in matlab,
 * except, of course, matlab is 1-indexed.
 * \param in a vector
 * \tparam FIRST first index to be copied
 * \tparam LAST last index to be copied
 * \return subvector
 */
template <int FIRST, int LAST, int ROWS, class T>
me::Vec<LAST - FIRST + 1, T> subvec(me::Vec<ROWS, T> const &in);

/**
 * Assign data in a vector v to a locations in a matrix corresponding to a range
 * of indices, in column-major order. The index range must be known at compile
 * time. So, assign<i1, i2>(A, v) is analogous
 * to A(i1:i2) = v in matlab
 * \param A a matrix
 * \tparam FIRST first index to be assigned
 * \tparam LAST last index to be assigned
 */
template <int FIRST, int LAST, int ROWS, int COLS, class T>
void assign(me::Mat<ROWS, COLS, T> &A, me::Vec<LAST - FIRST + 1, T> const &v);

/**
 * Assign data in a vector v to a locations in a matrix corresponding to
 * indices in a vector, ndx, in column-major order.
 * So, assign(A, v, ndx) is analogous to A(ndx) = v in matlab
 * \param A a matrix
 */
template <int ROWS, int COLS, int ROWS_VEC, class T>
void assign(me::Mat<ROWS, COLS, T> &in, me::Vec<ROWS_VEC, T> const &v, me::Vec<ROWS_VEC, unsigned int> const &ndx);

/**
 * Copies values in a vector, v, to a column of a matrix, A.
 * So, assignCol(A, n, v) is analogous to A(:, n) = v in matlab
 * \param A a matrix
 * \param n column index (zero based)
 * \param v vector of values to be copied
 */
template <int ROWS, int COLS, class T>
void assignCol(me::Mat<ROWS, COLS, T> &A, unsigned int n, me::Vec<ROWS, T> const &v);

/**
 * Copies values in a vector, v, to a row of a matrix, A.
 * So, assignRow(A, m, v) is analogous to A(m, :) = v in matlab
 * \param A a matrix
 * \param m row index (zero based)
 * \param v vector of values to be copied
 */
template <int ROWS, int COLS, class T>
void assignRow(me::Mat<ROWS, COLS, T> &A, unsigned int m, me::Vec<COLS, T> const &v);

/**
 * Copies values in a matrix row to a vector.
 * So, v = row(A, m) is analogous to v = A(m, :) in matlab
 * \param A a matrix
 * \param m row index (zero based)
 * \return vector of values in row m.
 */
template <int ROWS, int COLS, class T>
me::Vec<COLS, T> row(me::Mat<ROWS, COLS, T> &A, unsigned int m);

/**
 * Copies values in a matrix column to a vector.
 * So, v = col(A, n) is analogous to v = A(:, n) in matlab
 * \param A a matrix
 * \param n column index (zero based)
 * \return vector of values in column m
 */
template <int ROWS, int COLS, class T>
me::Vec<ROWS, T> col(me::Mat<ROWS, COLS, T> &A, unsigned int m);

/**
 * Generate a vector of unsigned ints going from FIRST to LAST.
 * Like a = FIRST:LAST in matlab.
 * \tparam FIRST first number in range
 * \tparam LAST last number in range
 * \return vector of unsigned ints
 */
template <unsigned int FIRST, unsigned int LAST>
me::Vec<LAST - FIRST + 1, unsigned int> span();

/**
 * Generate a vector of unsigned ints going from first to first + LENGTH-1.
 * Like a = FIRST:LAST in matlab.
 * \tparam FIRST first number in range
 * \tparam LAST last number in range
 * \return vector of unsigned ints
 */
template <unsigned int LENGTH>
me::Vec<LENGTH, unsigned int> span(unsigned int first);

/**
 * Generate a matrix of zeros().
 * \tparam ROWS rows of output matrix
 * \tparam COLS cols of output matrix
 * \return zeros matrix
 */
template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> zeros();

////////////////////////////////////////////
// Implementations of templated functions //
////////////////////////////////////////////

template <int ROWS, int COLS, class T>
void printMat(me::Mat<ROWS, COLS, T> const &in, std::string const &header)
{
    if (!header.empty())
    {
        std::cout << header;
    }
    std::cout << "[";
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {
            std::cout << in(i, j) << " ";
        }
        if (i < ROWS - 1)
        {
            std::cout << ";...";
        }
        else
        {
            std::cout << "];";
        }
        std::cout << std::endl;
    }
}

template <int ROWS, class T>
void printMat(me::Vec<ROWS, T> const &in, std::string const &header = "")
{
    if (!header.empty())
    {
        std::cout << header;
    }
    for (int i = 0; i < ROWS; i++)
    {
        std::cout << in[i] << std::endl;
    }
}

template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> zeros()
{
    me::Mat<ROWS, COLS, T> mat;

    for (unsigned int i = 0; i < ROWS; i++)
    {
        for (unsigned int j = 0; j < COLS; j++)
        {
            mat(i, j) = T(0.0);
        }
    }
    return mat;
}

template <int ROWS, class T>
me::Vec<ROWS, T> zeros()
{
    me::Vec<ROWS, T> out;

    for (unsigned int i = 0; i < ROWS; i++)
    {
        out[i] = T(0.0);
    }
    return out;
}

template <int ROWS, int COLS, class T>
me::Vec<ROWS * COLS, T> vec(me::Mat<ROWS, COLS, T> const &A)
{
    me::Vec<ROWS * COLS, T> out;
    me::Mat<COLS, ROWS, T> At = A.transpose();
    std::memcpy(out.begin(), At.begin(), sizeof(T) * ROWS * COLS);
    return out;
}

template <int FIRST, int LAST, int ROWS, class T>
me::Vec<LAST - FIRST + 1, T> subvec(me::Vec<ROWS, T> const &in)
{
    const unsigned int outLength = LAST - FIRST + 1;
    me::Vec<outLength, T> out;
    std::memcpy(out.begin(), in.begin() + FIRST, sizeof(T) * outLength);
    return out;
}

template <int ROWS, int COLS, int ROWS_VEC, class T>
void assign(me::Mat<ROWS, COLS, T> &in, me::Vec<ROWS_VEC, T> const &v, me::Vec<ROWS_VEC, unsigned int> const &ndx)
{
    for (unsigned int i = 0; i < ROWS_VEC; i++)
    {
        unsigned int iRow = ndx[i] % ROWS;
        unsigned int iCol = ndx[i] / ROWS;
        in(iRow, iCol) = v[i];
    }
}

template <int FIRST, int LAST, int ROWS, int COLS, class T>
void assign(me::Mat<ROWS, COLS, T> &in, me::Vec<LAST - FIRST + 1, T> const &v)
{
    for (unsigned int i = 0; i < LAST - FIRST + 1; i++)
    {
        unsigned int iRow = (i + FIRST) % ROWS;
        unsigned int iCol = (i + FIRST) / ROWS;
        in(iRow, iCol) = v[i];
    }
}

template <unsigned int FIRST, unsigned int LAST>
me::Vec<LAST - FIRST + 1, unsigned int> span()
{
    me::Vec<LAST - FIRST + 1, unsigned int> out;
    for (unsigned int i = 0; i < LAST - FIRST + 1; i++)
    {
        out[i] = i + FIRST;
    }
    return out;
}

template <unsigned int LENGTH>
me::Vec<LENGTH, unsigned int> span(unsigned int first)
{
    me::Vec<LENGTH, unsigned int> out;
    for (unsigned int i = 0; i < LENGTH; i++)
    {
        out[i] = i + first;
    }
    return out;
}

template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> mat(const me::Vec<ROWS * COLS, T> &v)
{
    me::Mat<COLS, ROWS, T> outT(v);
    return outT.transpose();
}

template <int ROWS, int COLS, class T>
me::Mat<ROWS, COLS, T> mat(const T* v)
{
    me::Mat<COLS, ROWS, T> outT;
    std::memcpy(outT.begin(), v, COLS * ROWS * sizeof(T));
    return outT.transpose();
}

template <int ROWS, int COLS, class T>
void assignCol(me::Mat<ROWS, COLS, T> &A, unsigned int n, me::Vec<ROWS, T> const &v)
{
    unsigned int first;
    first = n * ROWS;
    assign(A, v, span<ROWS>(first));
}

template <int ROWS, int COLS, class T>
void assignRow(me::Mat<ROWS, COLS, T> &A, unsigned int m, me::Vec<COLS, T> const &v)
{
    A[m] = v;
}

template <int ROWS, int COLS, class T>
me::Vec<COLS, T> row(me::Mat<ROWS, COLS, T> &A, unsigned int m)
{
    return A[m];
}

template <int ROWS, int COLS, class T>
me::Vec<ROWS, T> col(me::Mat<ROWS, COLS, T> &A, unsigned int m)
{
    me::Vec<ROWS, T> out;
    for (unsigned int i = 0; i < ROWS; i++)
    {
        out[i] = A[m][i];
    }
    return out;
}

} // namespace CalibUtils