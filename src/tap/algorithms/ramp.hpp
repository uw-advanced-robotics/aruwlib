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

#ifndef TAPROOT_RAMP_HPP_
#define TAPROOT_RAMP_HPP_

#include <cstdint>

namespace tap
{
namespace algorithms
{
/**
 * An output value is incremented or decremented at every call to update
 * until target has been reached.
 *
 * This is very similar to modm's ramp except for one difference: rather than
 * setting the increment at the beginning, you set the increment each time,
 * which allows you to take into account systems where time increment is not
 * constant.
 */
class Ramp
{
public:
    /**
     * Create a ramp generator.
     *
     * @param initialValue The starting value
     */
    explicit Ramp(float initialValue = 0.0f);

    /// Sets the target and value to the passed in `val`.
    void reset(float val);

    /// Sets a new target ramp value.
    void setTarget(float target);

    /// Sets the value to `value`.
    void setValue(float value);

    /**
     * Updates the ramp by incrementing or decrementing the target by the increment.
     *
     * @note For expected results, call this function every controller iteration.
     * @note The increment should be based on the time.<br>
     *      For example, if you are ramping a motor angle and want a motor to turn at some
     *      `rotationSpeed` pass in the following: `(currtime - prevtime) * rotationSpeed`
     */
    void update(float increment);

    /// Returns the current value being generated by the ramp.
    float getValue() const;

    /// Returns true if the value == target.
    bool isTargetReached() const;

    /// Returns the target value (where the ramp generator will head torwards).
    float getTarget() const;

private:
    static constexpr float RAMP_EPSILON = 0.00000000001f;
    float target;        /// The value's end goal.
    float value;         /// The value to be incremented towards the target.
    bool targetReached;  /// Whether or not target and value have converged.
};  // class Ramp

}  // namespace algorithms

}  // namespace tap

#endif  // TAPROOT_RAMP_HPP_
