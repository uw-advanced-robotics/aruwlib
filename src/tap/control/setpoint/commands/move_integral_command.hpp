/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_ROTATE_COMMAND_HPP_
#define TAPROOT_ROTATE_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/command.hpp"

#include "../interfaces/integrable_setpoint_subsystem.hpp"

namespace tap::control::setpoint
{
/**
 * A command that aims to keep the an IntegrableSetpointSubsystem's setpoint at some target value
 * until the integral of the setpoint over time equals a specified value.
 *
 * Ends if the subsystem is offline or jammed.
 */
class MoveIntegralCommand : public tap::control::Command
{
public:
    /**
     * Config struct that the user passes into the MoveIntegralCommand's constructor.
     *
     * @attention targetIntegralChange and desiredSetpoint must have the same sign.
     */
    struct Config
    {
        /// The desired change with units `units * seconds` (units of setpoint integrated with
        /// respect to time).
        float targetIntegralChange;
        /// The desired setpoint in units
        float desiredSetpoint;
        /**
         * The difference between the current and desired value when the command will be considered
         * to be complete, in the integral of units.
         *
         * @attention This value must be >= 0
         */
        float setpointTolerance;
    };

    /**
     * @param[in] integrableSetpointSubsystem The subsystem associated with the move integral
     * command.
     * @param[in] config The move integral command's config struct, @see Config.
     */
    MoveIntegralCommand(
        IntegrableSetpointSubsystem& integrableSetpointSubsystem,
        const Config& config);

    const char* getName() const override { return "move integral command"; }

    inline bool isReady() override
    {
        return !integrableSetpointSubsystem.isJammed() && integrableSetpointSubsystem.isOnline();
    }

    void initialize() override;

    void execute() override {}

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    Config config;

    IntegrableSetpointSubsystem& integrableSetpointSubsystem;

    float finalTargetPosition = 0;

    /// @return True if the intgral setpoint has reached the target integral
    bool targetIntegralReached() const
    {
        if (config.targetIntegralChange > 0)
        {
            return integrableSetpointSubsystem.getCurrentValueIntegral() >
                   finalTargetPosition - config.setpointTolerance;
        }
        else
        {
            return integrableSetpointSubsystem.getCurrentValueIntegral() <
                   finalTargetPosition + config.setpointTolerance;
        }
    }
};  // class MoveIntegralCommand

}  // namespace tap::control::setpoint

#endif  // TAPROOT_ROTATE_COMMAND_HPP_
