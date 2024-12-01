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

#ifndef VELOCITY_PID_HPP_
#define VELOCITY_PID_HPP_

#include <cstdint>

namespace tap::algorithms
{
struct PIDConfig
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float maxICumulative = 0.0f;
    float maxOutput = 0.0f;
};

class VelocityPID
{
public:
    VelocityPID(const PIDConfig &pidConfig);

    /**
     * @brief Runs the PID controller, calculating a new output value.
     *
     * @param error Measured error between target and actual value
     * @param dt    Time since last call to update
     * @return float Computed output value
     */
    float update(float error, float dt);

    /**
     * @brief Returns the calculated output
     *
     * @return float
     */
    inline float getOutput() const;

    // Resets all values
    inline void reset();

private:
    PIDConfig pidConfig;

    float errorSum = 0.0f;
    float output = 0.0f;
    float lastError = 0.0f;
};

}  // namespace tap::algorithms

#endif  // VELOCITY_PID_HPP_
