/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "math_user_utils.hpp"

#include <cstdint>

float tap::algorithms::fastInvSqrt(float x)
{
    static_assert(sizeof(float) == 4, "fast inverse sqrt requires 32-bit float");
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = reinterpretCopy<float, int32_t>(y);
    i = 0x5f3759df - (i >> 1);
    y = reinterpretCopy<int32_t, float>(i);
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void tap::algorithms::rotateVector(float* x, float* y, float radians)
{
    float x_temp = *x;
    *x = (*x) * cosf(radians) - *y * sinf(radians);
    *y = x_temp * sinf(radians) + *y * cosf(radians);
}

tap::algorithms::CMSISMat<3, 1> tap::algorithms::cross(
    const tap::algorithms::CMSISMat<3, 1>& a,
    const tap::algorithms::CMSISMat<3, 1>& b)
{
    return tap::algorithms::CMSISMat<3, 1>(
        {a.data[1] * b.data[2] - a.data[2] * b.data[1],
         a.data[2] * b.data[0] - a.data[0] * b.data[2],
         a.data[0] * b.data[1] - a.data[1] * b.data[0]});
}

tap::algorithms::CMSISMat<3, 3> tap::algorithms::fromEulerAngles(
    const float roll,
    const float pitch,
    const float yaw)
{
    return tap::algorithms::CMSISMat<3, 3>(
        {cosf(yaw) * cosf(pitch),
         (cosf(yaw) * sinf(pitch) * sinf(roll)) - (sinf(yaw) * cosf(roll)),
         (cosf(yaw) * sinf(pitch) * cosf(roll)) + sinf(yaw) * sinf(roll),
         sinf(yaw) * cosf(pitch),
         sinf(yaw) * sinf(pitch) * sinf(roll) + cosf(yaw) * cosf(roll),
         sinf(yaw) * sinf(pitch) * cosf(roll) - cosf(yaw) * sinf(roll),
         -sinf(pitch),
         cosf(pitch) * sinf(roll),
         cosf(pitch) * cosf(roll)});
}

modm::Vector3f tap::algorithms::eulerAnglesFromQuaternion(modm::Quaternion<float>& q)
{
    modm::Vector3f eulerAngles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    eulerAngles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    float cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    eulerAngles.y = 2 * std::atan2(sinp, cosp) - static_cast<float>(M_PI) / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    eulerAngles.z = std::atan2(siny_cosp, cosy_cosp);

    return eulerAngles;
}
