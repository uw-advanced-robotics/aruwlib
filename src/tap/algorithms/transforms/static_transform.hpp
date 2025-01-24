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

#ifndef TAPROOT_DYNAMIC_TRANSFORM_HPP_
#define TAPROOT_DYNAMIC_TRANSFORM_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#include "dynamic_transform.hpp"
#include "position.hpp"

namespace tap::algorithms::transforms
{
// TODO: somewhat inaccurate name since rotational frames are not inertial
class StaticTransform : protected DynamicTransform
{
public:
    StaticTransform(
        const Transform transform,
        float xVel,
        float yVel,
        float zVel,
        float rollVel,
        float pitchVel,
        float yawVel)
        : DynamicTransform(transform),
          transVel({xVel, yVel, zVel}),
          angVel({xVel, yVel, zVel})
    {
    }

    StaticTransform(
        const Transform transform,
        const CMSISMat<3, 1>& transVel,
        const CMSISMat<3, 1>& angVel)
        : DynamicTransform(transform),
          transVel(transVel),
          angVel(angVel)
    {
    }

    StaticTransform(const Transform transform, CMSISMat<3, 1>&& transVel, CMSISMat<3, 1>&& angVel)
        : DynamicTransform(transform),
          transVel(std::move(transVel)),
          angVel(std::move(angVel))
    {
    }

    inline StaticTransform(
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
        : DynamicTransform(x, y, z, roll, pitch, yaw),
          transVel({xVel, yVel, zVel}),
          angVel({rollVel, pitchVel, yawVel})
    {
    }

    using DynamicTransform::apply;
    using DynamicTransform::getRotation;
    using DynamicTransform::getTranslation;
    using DynamicTransform::updateRotation;
    using DynamicTransform::updateTranslation;

    inline void updateTransVel(Vector transVel) { this->transVel = transVel.coordinates(); }

    inline void updateTransVel(const float x, const float y, const float z)
    {
        this->transVel = CMSISMat<3, 1>({x, y, z});
    }

    Vector apply(const Position& position, const Vector& velocity) const;

    StaticTransform getInverse() const;

    StaticTransform compose(const StaticTransform& second) const;

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
};  // class StaticTransform

}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_DYNAMIC_TRANSFORM_HPP_