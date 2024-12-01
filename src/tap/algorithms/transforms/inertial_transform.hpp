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

#ifndef TAPROOT_INERTIAL_TRANSFORM_HPP_
#define TAPROOT_INERTIAL_TRANSFORM_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#include "position.hpp"
#include "transform.hpp"

namespace tap::algorithms::transforms
{
// TODO: somewhat inaccurate name since rotational frames are not inertial
class InertialTransform : protected Transform
{
public:
    InertialTransform(
        const Transform transform,
        float xVel,
        float yVel,
        float zVel,
        float rollVel,
        float pitchVel,
        float yawVel)
        : Transform(transform),
          transVel({xVel, yVel, zVel}),
          angVel({xVel, yVel, zVel})
    {
    }

    InertialTransform(
        const Transform transform,
        const CMSISMat<3, 1>& transVel,
        const CMSISMat<3, 1>& angVel)
        : Transform(transform),
          transVel(transVel),
          angVel(angVel)
    {
    }

    InertialTransform(const Transform transform, CMSISMat<3, 1>&& transVel, CMSISMat<3, 1>&& angVel)
        : Transform(transform),
          transVel(std::move(transVel)),
          angVel(std::move(angVel))
    {
    }

    inline InertialTransform(
        float x,
        float y,
        float z,
        float roll,
        float pitch,
        float yaw,
        float xVel,
        float yVel,
        float zVel,
        float rollVel,
        float pitchVel,
        float yawVel)
        : Transform(x, y, z, roll, pitch, yaw),
          transVel({xVel, yVel, zVel}),
          angVel({rollVel, pitchVel, yawVel})
    {
    }

    using Transform::apply;
    using Transform::getRotation;
    using Transform::getTranslation;
    using Transform::updateRotation;
    using Transform::updateTranslation;

    inline void updateTransVel(Vector transVel) { this->transVel = transVel.coordinates(); }

    inline void updateTransVel(const float x, const float y, const float z)
    {
        this->transVel = CMSISMat<3, 1>({x, y, z});
    }

    Vector apply(const Position& position, const Vector& velocity) const;

    InertialTransform getInverse() const;

    InertialTransform compose(const InertialTransform& second) const;

private:
    /**
     * Translation differential vector.
     *
     * The velocity of the target frame origin in the source frame.
     */
    CMSISMat<3, 1> transVel;

    /**
     * Differential of rotation matrix.
     *
     * The angular velocity of the target frame coordinates in the source frame.
     */
    CMSISMat<3, 1> angVel;
};  // class InertialTransform

}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_INERTIAL_TRANSFORM_HPP_