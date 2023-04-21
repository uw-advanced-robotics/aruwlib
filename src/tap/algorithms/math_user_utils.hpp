/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_MATH_USER_UTILS_HPP_
#define TAPROOT_MATH_USER_UTILS_HPP_

#include <array>
#include <cinttypes>
#include <cmath>
#include <cstring>

#include "modm/architecture/interface/assert.hpp"
#include "modm/math/geometry/angle.hpp"

namespace tap
{
namespace algorithms
{
/// Acceleration due to gravity, in m/s^2.
static constexpr float ACCELERATION_GRAVITY = 9.80665f;

/**
 * Use this instead of the == operator when asserting equality for floats.
 * Performs \code fabsf(val1-val2)<=epsilon\endcode
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
    return fabsf(val1 - val2) <= epsilon;
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
inline T limitVal(T val, T min, T max)
{
    if (min > max)
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
 * \f$y_{filtered} = \alpha \cdot y_{n+1} + (1-\alpha) \cdot y_n\f$
 *
 * Here is a simple use case. To use the low pass filter, pass
 * in the val you are low passing in as the first parameter and
 * have that value accept what lowPassFilter returns.
 * \code
 * val = lowPassFilter(val, newValue, 0.1f);
 * \endcode
 *
 * @note Only use this if you are willing to introduce some lag into
 *      your system, and be careful if you do.
 * @param[in] prevValue The previous low passed value.
 * @param[in] newValue The new data to be passed into the low pass filter.
 * @param[in] alpha The amount of smoothing. The larger the alpha, the
 *      less smoothing occurs. An alpha of 1 means that you want to favor
 *      the newValue and thus is not doing any filtering. Must be between
 *      [0, 1].
 * @return The newly low passed filter data.
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
    memcpy(static_cast<void*>(&result), static_cast<void*>(&from), sizeof(To));
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
void rotateVector(float* x, float* y, float radians);

/**
 * Constexpr ceil
 * (https://stackoverflow.com/questions/31952237/looking-for-a-constexpr-ceil-function).
 */
constexpr int32_t ceil(float num)
{
    return (static_cast<float>(static_cast<int32_t>(num)) == num)
               ? static_cast<int32_t>(num)
               : static_cast<int32_t>(num) + ((num > 0) ? 1 : 0);
}

/**
 * Returns the sign of the value passed in. Either -1, 0, or 1. Works for all base types and any
 * types that implement construction from an int. Faster than copysign.
 */
template <typename T>
int getSign(T val)
{
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief Bilinear Interpolation of a regularly-spaced grid of values.
 * Let x = dimension 1 and y = dimension 2 of the 2D array of values
 * @param values 2D-array pointer of f(x,y) values
 * @return approximation of values at (xdes,ydes)
 */
template <typename T, size_t xSize, size_t ySize>
float interpolateLinear2D(
    std::array<std::array<T, ySize>, xSize>& values,
    const float xMin,
    const float xMax,
    const float dx,
    const float yMin,
    const float yMax,
    const float dy,
    float xDes,
    float yDes)
{
    modm_assert((xMax - xMin) / dx == xSize - 1, "Bilinear Interpolator", "x range error");
    modm_assert((yMax - yMin) / dy == ySize - 1, "Bilinear Interpolator", "y range error");
    float xDesBounded = limitVal(xDes, xMin, xMax);  // no extrapolation allowed
    float yDesBounded = limitVal(yDes, yMin, yMax);  // no extrapolation allowed

    // get the desired pt, normalized such that it represents the index of it, then
    // Get nearest pt which is less
    int xIndex = floor((xDesBounded - xMin) / dx);
    xIndex = limitVal(xIndex, 0, static_cast<int>(xSize) - 2);  // Prevent OOB errors
    float x1 = xMin + xIndex * dx;                              // gets value from index
    float x2 = xMin + (xIndex + 1) * dx;

    int yIndex = floor((yDesBounded - yMin) / dy);
    yIndex = limitVal(yIndex, 0, static_cast<int>(ySize) - 2);
    float y1 = yMin + yIndex * dy;
    float y2 = yMin + (yIndex + 1) * dy;

    float q11, q12, q21, q22;  // values of x1y1, x1y2, x2y1, x2y2
    q11 = static_cast<float>(values.at(xIndex).at(yIndex));
    q12 = static_cast<float>(values.at(xIndex).at(yIndex + 1));
    q21 = static_cast<float>(values.at(xIndex + 1).at(yIndex));
    q22 = static_cast<float>(values.at(xIndex + 1).at(yIndex + 1));

    float x2x, y2y, yy1, xx1;  // deltas from each pt to sample pt
    x2x = x2 - xDesBounded;
    y2y = y2 - yDesBounded;
    yy1 = yDesBounded - y1;
    xx1 = xDesBounded - x1;
    // it's essentially a weighted average
    return 1.0 / (dx * dy) *
           (q11 * x2x * y2y + q21 * xx1 * y2y + q12 * x2x * yy1 + q22 * xx1 * yy1);
}

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_MATH_USER_UTILS_HPP_
