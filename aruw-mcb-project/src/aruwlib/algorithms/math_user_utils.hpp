/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MATH_USER_UTILS_HPP_
#define MATH_USER_UTILS_HPP_

#include <cmath>
#include <cstring>

#include <modm/math/matrix.hpp>

namespace aruwlib
{
namespace algorithms
{
///< An approximation for pi.
constexpr float PI = 3.1415926535897932384626f;

///< Floating point conversion from degrees to radians using PI.
inline float degreesToRadians(float degrees) { return degrees * PI / 180.0f; }

///< Floating point conversion from radians to degrees using PI.
inline float radiansToDegrees(float radians) { return radians * 180.f / PI; }

/**
 * Use this instead of the == operator when asserting equality for floats.
 * Performs \code fabsf(val1-val2)<epsilon\endcode
 *
 * @param[in] val1 the first value to compare.
 * @param[in] val2 the second value to compare.
 * @param[in] epsilon the floating point equality tolerance, for equality a
 *      recommended epsilon is 1E-6, though any small epsilon will do.
 * @return true if val1 and val2 are equal within some epsilon tolerance,
 *      false otherwise.
 */
inline bool compareFloatClose(float val1, float val2, float epsilon)
{
    return fabsf(val1 - val2) < epsilon;
}

/**
 * Limits the value between some min an max (between [min, max]).
 *
 * @tparam T the type you would like to limit.
 * @param[in] val the value to limit.
 * @param[in] min the min that val will be limited to.
 * @param[in] max the max that val will be limited to.
 * @return the limited value.
 */
template <typename T>
T limitVal(T val, T min, T max)
{
    if (min >= max)
    {
        return val;
    }
    if (val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    else
    {
        return val;
    }
}

/**
 * A simple floating point low pass filter, e.g.
 * \f$y_{filtered} = \alpha * y_{n+1} + (1-\alpha) \cdot y_n\f$
 *
 * Here is a simple use case. To use the low pass filter, pass
 * in the val you are low passing in as the first parameter and
 * have that value accept what lowPassFilter returns.
 * \code
 * val = lowPassFilter(val, newValue, 0.1f);
 * \endcode
 *
 * @note only use this if you are willing to introduce some lag into
 *      your system, and be careful if you do.
 * @param[in] prevValue the previous low passed value.
 * @param[in] newValue the new data to be passed into the low pass filter.
 * @param[in] alpha the amount of smoothing. The larger the alpha, the
 *      less smoothing occurs. An alpha of 1 means that you want to favor
 *      the newValue and thus is not doing any filtering. Must be between
 *      [0, 1].
 * @return the newly low passed filter data.
 */
inline float lowPassFilter(float prevValue, float newValue, float alpha)
{
    if (alpha < 0.0f || alpha > 1.0f)
    {
        return newValue;
    }
    return alpha * newValue + (1.0f - alpha) * prevValue;
}

template <typename From, typename To>
To reinterpretCopy(From from)
{
    static_assert(sizeof(From) == sizeof(To), "can only reinterpret-copy types of the same size");
    To result;
    memcpy(static_cast<void *>(&result), static_cast<void *>(&from), sizeof(To));
    return result;
}

/**
 * Fast inverse square-root, to calculate 1/Sqrt(x).
 *
 * @param[in] input:x
 * @retval    1/Sqrt(x)
 */
float fastInvSqrt(float x);

/**
 * Performs a rotation matrix on the given x and y components of a vector.
 *
 * @param x the x component of the vector to be rotated.
 * @param y the y component of the vector to be rotated.
 * @param angle the angle by which to rotate the vector <x, y>, in radians.
 * @retval none.
 */
void rotateVector(float *x, float *y, float radians);

/**
 * Copies the matrix in `in` to the matrix in `out`, eliminating row `rowToElim` and
 * column `colToElim`.
 *
 * @tparam T The type stored in the matrix being manipulated.
 * @tparam ROWS The number of rows in the matrix.
 * @tparam COLUMNS The number of columns in the matrix.
 * @param[in] in The matrix to copy. Must be a square matrix.
 * @param[out] out The location where the matrix will be copied to.
 * @param[in] rowToElim The row to eliminate during copying.
 * @param[in] colToElim The column to eliminate during copying.
 * @return `true` if copying succeeded, `false` otherwise`.
 * @note does not check if rowToElim or colToElim is out of bounds.
 */
template <typename T, uint8_t ROWS, uint8_t COLUMNS>
inline bool copyRowColElim(
    const modm::Matrix<T, ROWS, COLUMNS> &in,
    modm::Matrix<T, ROWS - 1, COLUMNS - 1> *out,
    int rowToElim,
    int colToElim)
{
    static_assert(ROWS == COLUMNS, "rows must be equal to columns to compute the adjoint");

    if (out == nullptr)
    {
        return false;
    }

    if (ROWS == 1)
    {
        return true;
    }

    // void getCfactor(int M[N][N], int t[N][N], int p, int q, int n) {
    int i = 0, j = 0;
    for (int r = 0; r < ROWS; r++)
    {
        // Copy only those elements which are not in given row r and column c:
        for (int c = 0; c < COLUMNS; c++)
        {
            if (r != rowToElim && c != colToElim)
            {
                (*out)[i][j++] = in[r][c];  // If row is filled increase r index and reset c index
                if (j == ROWS - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
    return true;
}

/**
 * Finds the adjoint matrix of `in` and puts it in `out`.
 *
 * @tparam T The type stored in the matrix being manipulated.
 * @tparam ROWS The number of rows in the matrix.
 * @tparam COLUMNS The number of columns in the matrix.
 * @param in[in] The matrix to be used in computing the adjoint set. Must be a square matrix.
 * @param out[out] A return parameter wherethe inverse matrix will be stored.
 * @return `true` on success, `false` on failure.
 */
template <typename T, uint8_t ROWS, uint8_t COLUMNS>
inline bool adjoint(const modm::Matrix<T, ROWS, COLUMNS> &in, modm::Matrix<T, ROWS, COLUMNS> *out)
{
    static_assert(ROWS == COLUMNS, "rows must be equal to columns to compute the adjoint");

    if (out == nullptr)
    {
        return false;
    }

    if (ROWS == 1)
    {
        (*out)[0][0] = 1;
        return true;
    }

    // temp is used to store copyRowColElims of the in matrix
    int sign = 1;
    modm::Matrix<T, ROWS - 1, COLUMNS - 1> temp;

    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLUMNS; j++)
        {
            // Get copyRowColElim of A[i][j]
            if (!copyRowColElim(in, &temp, i, j))
            {
                return false;
            }

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            (*out)[j][i] = temp.determinant() * sign;
        }
    }
    return true;
}

/**
 * Takes the inverse of `in` and puts the result in `out`.
 *
 * @tparam T The type stored in the matrix being manipulated.
 * @tparam ROWS The number of rows in the matrix.
 * @tparam COLUMNS The number of columns in the matrix.
 * @param in[in] The matrix to be taken an inverse of. Must be a square matrix.
 * @param out[out] A return parameter wherethe inverse matrix will be stored.
 * @return `true` on success, `false` on failure.
 */
template <typename T, uint8_t ROWS, uint8_t COLUMNS>
inline bool inverse(const modm::Matrix<T, ROWS, COLUMNS> &in, modm::Matrix<T, ROWS, COLUMNS> *out)
{
    static_assert(ROWS == COLUMNS, "rows must be equal to column to convert the inverse");

    if (out == nullptr)
    {
        return false;
    }

    // Find adjiont
    adjoint(in, out);

    // Find determinant
    T det = in.determinant();

    if (det == 0)
    {
        return false;
    }

    // Inverse is adjoint dividided by determinant
    (*out) /= det;
    return true;
}

}  // namespace algorithms

}  // namespace aruwlib

#endif  // MATH_USER_UTILS_HPP_
