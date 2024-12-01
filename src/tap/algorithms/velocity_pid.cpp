/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "velocity_pid.hpp"

#include <algorithm>
#include <cmath>

namespace tap::algorithms
{

VelocityPID::VelocityPID(const PIDConfig &pidConfig) : pidConfig(pidConfig) { reset(); }

float VelocityPID::getOutput() const { return output; }

void VelocityPID::reset()
{
    output = 0.0f;
    errorSum = 0.0f;
    lastError = 0.0f;
}

float VelocityPID::update(float error, float dt)
{
    // If no time has passed, don't update
    if (dt == 0.0f)
    {
        return output;
    }

    // Proportional term
    float pTerm = pidConfig.kp * error;

    // Integral term
    // If pTerm saturates the output, don't add to I term
    if (std::fabs(pTerm) < pidConfig.maxOutput)
    {
        errorSum += error * dt;
        errorSum = std::clamp(errorSum, -pidConfig.maxICumulative, pidConfig.maxICumulative);
    }

    float iTerm = pidConfig.ki * errorSum;

    // Derivative term
    float dTerm = pidConfig.kd * (error - lastError) / dt;

    // Compute output
    output = output + (pTerm + iTerm + dTerm);
    output = std::clamp(output, -pidConfig.maxOutput, pidConfig.maxOutput);

    // Update prior error
    lastError = error;

    return output;
}

}  // namespace tap::algorithms
