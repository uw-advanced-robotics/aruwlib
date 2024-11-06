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

#include "ballistics.hpp"

#include "math_user_utils.hpp"

namespace tap::algorithms::ballistics
{
bool computeTravelTime(
    const units::Vector3Position &targetPosition,
    units::LinearVelocity<> bulletVelocity,
    units::Time<> *travelTime,
    units::Angle<> *turretPitch,
    const units::Length<> pitchAxisOffset)
{
    units::Length horizontalDist =
        units::math::hypot(targetPosition.x, targetPosition.y) + pitchAxisOffset;
    units::Exponentiated<units::LinearVelocity<>, ratio<2>> bulletVelocitySquared =
        units::math::square(bulletVelocity);
    units::Exponentiated<units::LinearVelocity<>, ratio<4>> sqrtTerm =
        units::math::square(bulletVelocitySquared) -
        ACCELERATION_GRAVITY * (ACCELERATION_GRAVITY * units::math::square(horizontalDist) +
         2 * targetPosition.z *
          bulletVelocitySquared);  // todo: make sure this is correct

    if (units::math::sgn(sqrtTerm) < 0)
    {
        return false;
    }

    // Equation obtained from the wikipedia page on projectile motion
    *turretPitch = -units::math::atan2(
        bulletVelocitySquared - units::math::sqrt(sqrtTerm),
        (ACCELERATION_GRAVITY * horizontalDist));

    // For vertical aiming, y_f = v_0*t - 0.5*g*t^2 -> t = (v_0 - sqrt((v_0)^2 - 2*g*y_f))/g
    // We use the negative root since the collision will happen on the first instance that the
    // trajectory reaches y_f
    if (units::math::compareClose(
            *turretPitch,
            units::Angle<>(0),
            units::Angle<>(1E-2)))  // todo create Quantity replacement for compareFloatClose
    {
        tap::units::Exponentiated<tap::units::LinearVelocity<>, ratio<2>> sqrtTerm =
            units::math::square(bulletVelocity) - 2 * ACCELERATION_GRAVITY * targetPosition.z;

        // If there isn't a real-valued root, there is no time where we can reach the target with
        // the given assumptions
        if (units::math::sgn(sqrtTerm) < 0)
        {
            return false;
        }

        *travelTime = (bulletVelocity - units::math::sqrt(sqrtTerm)) / ACCELERATION_GRAVITY;
        return true;
    }

    // Equation obtained from the wikipedia page on projectile motion
    *travelTime = horizontalDist / (bulletVelocity * units::math::cos(*turretPitch));

    return !isnan(turretPitch->valueOf()) && !isnan(travelTime->valueOf());
}

bool findTargetProjectileIntersection(
    const AbstractKinematicState &targetInitialState,
    units::LinearVelocity<> bulletVelocity,
    uint8_t numIterations,
    units::Angle<> *turretPitch,
    units::Angle<> *turretYaw,
    units::Time<> *projectedTravelTime,
    const units::Length<> pitchAxisOffset)
{
    units::Vector3Position projectedTargetPosition =
        targetInitialState.projectForward(units::Time<>(0));

    if (projectedTargetPosition.x == units::Length(0) &&
        projectedTargetPosition.y == units::Length(0) &&
        projectedTargetPosition.z == units::Length(0))
    {
        return false;
    }

    for (int i = 0; i < numIterations; i++)
    {
        if (!computeTravelTime(
                projectedTargetPosition,
                bulletVelocity,
                projectedTravelTime,
                turretPitch,
                pitchAxisOffset))
        {
            return false;
        }
        projectedTargetPosition = targetInitialState.projectForward(*projectedTravelTime);
    }

    *turretYaw = units::math::atan2(projectedTargetPosition.y, projectedTargetPosition.x);

    return !isnan(turretPitch->valueOf()) && !isnan(turretYaw->valueOf());
}

}  // namespace tap::algorithms::ballistics
