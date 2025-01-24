/*
 * Copyright (c) 2022-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "static_transform.hpp"

namespace tap::algorithms::transforms
{

// TODO: garbled mess
Vector StaticTransform::apply(const Position& position, const Vector& velocity) const
{
    // TODO: INFINITELY CURSED
    // First add the extra velocities induced by angular/translational velocity then rotate like a
    // vector
    return Transform::apply(
        Vector(velocity.coordinates() - transVel - cross(angVel, position.coordinates())));
}

StaticTransform StaticTransform::getInverse() const
{
    return StaticTransform(Transform::getInverse(), -transVel, -angVel);
}

StaticTransform StaticTransform::compose(const StaticTransform& second) const
{
    CMSISMat<3, 1> transVel = this->transVel + this->getRotation().matrix() * second.transVel +
                              cross(this->angVel, second.getTranslation().coordinates());
    CMSISMat<3, 1> angVel = this->transVel + second.transVel;
    return StaticTransform(Transform::compose(second), transVel, angVel);
}

}  // namespace tap::algorithms::transforms