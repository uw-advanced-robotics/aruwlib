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

#include "unjam_integral_command.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace tap::control::setpoint
{
UnjamIntegralCommand::UnjamIntegralCommand(
    IntegrableSetpointSubsystem& integrableSetpointSubsystem,
    const Config& config)
    : integrableSetpointSubsystem(integrableSetpointSubsystem),
      config(config)
{
    assert(config.targetUnjamIntegralChange > 0);
    assert(config.targetCycleCount > 0);
    assert(config.maxWaitTime > 0);

    // max wait time must be > min time it will take to reach the unjam displacement given the unjam
    // velocity
    assert(
        1000.0f * (this->config.targetUnjamIntegralChange / this->config.unjamSetpoint) <
        this->config.maxWaitTime);

    addSubsystemRequirement(&integrableSetpointSubsystem);

    unjamRotateTimeout.stop();
}

bool UnjamIntegralCommand::isReady() { return integrableSetpointSubsystem.isOnline(); }

void UnjamIntegralCommand::initialize()
{
    unjamRotateTimeout.restart(config.maxWaitTime);

    positionBeforeUnjam = integrableSetpointSubsystem.getCurrentValueIntegral();

    forwardsCleared = false;
    backwardsCleared = false;
    backwardsCount = 0;

    beginUnjamBackwards();
}

void UnjamIntegralCommand::execute()
{
    float curPosition = integrableSetpointSubsystem.getCurrentValueIntegral();

    switch (currUnjamState)
    {
        case UNJAM_BACKWARD:
            if (curPosition <= positionBeforeUnjam - config.targetUnjamIntegralChange)
            {
                backwardsCleared = true;
                beginUnjamForwards();
            }
            else if (unjamRotateTimeout.isExpired())
            {
                beginUnjamForwards();
            }
            break;
        case UNJAM_FORWARD:
            if (curPosition >= positionBeforeUnjam)
            {
                forwardsCleared = true;
                beginUnjamBackwards();
            }
            else if (unjamRotateTimeout.isExpired())
            {
                beginUnjamBackwards();
            }
            break;
        case JAM_CLEARED:
            break;
    }

    // Forward and backward thresholds cleared, try to return to original setpoint.
    if (currUnjamState != JAM_CLEARED && forwardsCleared && backwardsCleared)
    {
        currUnjamState = JAM_CLEARED;
    }
}

void UnjamIntegralCommand::end(bool)
{
    if (currUnjamState == JAM_CLEARED)
    {
        integrableSetpointSubsystem.clearJam();
    }
    integrableSetpointSubsystem.setSetpoint(0);
}

bool UnjamIntegralCommand::isFinished() const
{
    return !integrableSetpointSubsystem.isOnline() || (currUnjamState == JAM_CLEARED) ||
           (backwardsCount >= config.targetCycleCount + 1);
}

void UnjamIntegralCommand::beginUnjamForwards()
{
    unjamRotateTimeout.restart(config.maxWaitTime);
    integrableSetpointSubsystem.setSetpoint(config.unjamSetpoint);
    currUnjamState = UNJAM_FORWARD;
}

void UnjamIntegralCommand::beginUnjamBackwards()
{
    unjamRotateTimeout.restart(config.maxWaitTime);
    integrableSetpointSubsystem.setSetpoint(-config.unjamSetpoint);
    currUnjamState = UNJAM_BACKWARD;
    backwardsCount += 1;
}

}  // namespace tap::control::setpoint
