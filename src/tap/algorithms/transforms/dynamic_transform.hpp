/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef TAPROOT_TRANSFORM_HPP_
#define TAPROOT_TRANSFORM_HPP_

#include "tap/algorithms/math_user_utils.hpp"

#include "orientation.hpp"
#include "position.hpp"
#include "vector.hpp"

namespace tap::algorithms::transforms
{
/**
 Represents a transformation from one coordinate frame to another.

    A DynamicTransform from frame A to frame B defines a relationship between the two frames, such
 that a spatial measurement in frame A can be represented equivalently in frame B by applying a
    translational and rotational offset. This process is known as *applying* a transform.

    DynamicTransforms are specified as a translation and rotation of some "target" frame relative to
 some "source" frame. The "translation" is the target frame's origin in source frame, and the
    "rotation" is the target frame's orientation relative to the source frame's orientation.

    Conceptually, translations are applied "before" rotations. This means that the origin of the
    target frame is entirely defined by the translation in the source frame, and the rotation serves
    only to change the orientation of the target frame's axes relative to the source frame.

    Utilizes arm's CMSIS matrix operations.

    @param SOURCE represents the source frame of the transformation.
    @param TARGET represents the target frame of the transformation.
 */
class DynamicTransform
{
public:
    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational åcceleration of this transformation.
     * @param angularVelocity Angular velocity pseudovector of this transformation.
     */
    // DynamicTransform(
    //     const Position& translation,
    //     const Orientation& rotation,
    //     const Vector& velocity,
    //     const Vector& acceleration,
    //     const Vector& angularVelocity);

    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational åcceleration of this transformation.
     * @param angularVelocity Angular velocity pseudovector of this transformation.
     */
    // DynamicTransform(
    //     Position&& translation,
    //     Orientation&& rotation,
    //     Vector&& velocity,
    //     Vector&& acceleration,
    //     Vector&& angularVelocity);

    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational åcceleration of this transformation.
     * @param angularVelocity Angular velocity pseudovector of this transformation.
     */
    DynamicTransform(
        const CMSISMat<3, 1>& translation,
        const CMSISMat<3, 3>& rotation,
        const CMSISMat<3, 1>& velocity,
        const CMSISMat<3, 1>& acceleration,
        const CMSISMat<3, 3>& angularVelocity);

    /**
     * @param translation Initial translation of this transformation.
     * @param rotation Initial rotation of this transformation.
     * @param velocity Translational velocity of this transformation.
     * @param acceleration Translational åcceleration of this transformation.
     * @param angularVelocity Angular velocity pseudovector of this transformation.
     */
    DynamicTransform(
        CMSISMat<3, 1>&& translation,
        CMSISMat<3, 3>&& rotation,
        CMSISMat<3, 1>&& velocity,
        CMSISMat<3, 1>&& acceleration,
        CMSISMat<3, 3>&& angularVelocity);

    /**
     * Constructs rotations using XYZ Euler angles,
     * so rotations are applied in order of A, B, then C.
     * As an example, for an x-forward, z-up coordinate system,
     * this is in the order of roll, pitch, then yaw.
     *
     * @param x: Initial x-component of the translation.
     * @param y: Initial y-component of the translation.
     * @param z: Initial z-component of the translation.
     * @param A: Initial rotation angle about the x-axis.
     * @param B: Initial rotation angle about the y-axis.
     * @param C: Initial rotation angle about the z-axis.
     */
    DynamicTransform(
        float x,
        float y,
        float z,
        float vx,
        float vy,
        float vz,
        float ax,
        float ay,
        float az,
        float roll,
        float pitch,
        float yaw,
        float rollVel,
        float pitchVel,
        float yawVel);

    // TODO: template specialization for transform between identical frames??
    /**
     * Constructs an identity transform.
     */
    static inline DynamicTransform identity()
    {
        return DynamicTransform(0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.);
    }

    /**
     * Apply this transform to a position.
     *
     * @param[in] position Position in source frame.
     * @return Position in target frame.
     */
    Position apply(const Position& position) const;

    /**
     * Rotates a vector in the source frame to a vector in the target frame.
     *
     * Intended to be used for things like velocities and accelerations which represent the
     * difference between two positions in space, since both positions get translated the same way,
     * causing the translation to cancel out.
     *
     * @param vector Vector as read by source frame.
     * @return Vector in target frame's basis.
     */
    Vector apply(const Vector& vector) const;

    /**
     *
     */
    Orientation apply(const Orientation& orientation) const;

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param newTranslation updated position of target in source frame.
     */
    inline void updateTranslation(const Position& newTranslation)
    {
        this->translation = newTranslation.coordinates();
    }

    inline void updateTranslation(Position&& newTranslation)
    {
        this->translation = std::move(newTranslation.coordinates());
    }

    /**
     * Updates the translation of the current transformation matrix.
     *
     * @param x new translation x-component.
     * @param y new translation y-component.
     * @param z new translation z-component.
     */
    inline void updateTranslation(float x, float y, float z)
    {
        this->translation = CMSISMat<3, 1>({x, y, z});
    }

    /**
     * Updates the rotation of the current transformation matrix.
     *
     * @param newRotation updated orienation of target frame in source frame.
     */
    inline void updateRotation(const Orientation& newRotation)
    {
        this->rotation = newRotation.matrix();
        this->tRotation = this->rotation.transpose();
    }

    inline void updateRotation(Orientation&& newRotation)
    {
        this->rotation = std::move(newRotation.matrix());
        this->tRotation = this->rotation.transpose();
    }

    /**
     * Updates the rotation of the current transformation matrix.
     * Takes rotation angles in the order of roll->pitch->yaw.
     *
     * @param roll updated rotation angle about the x-axis.
     * @param pitch updated rotation angle about the y-axis.
     * @param yaw updated rotation angle about the z-axis.
     */
    void updateRotation(float roll, float pitch, float yaw)
    {
        this->rotation = Orientation(roll, pitch, yaw).matrix();
        this->tRotation = this->rotation.transpose();
    }

    /**
     * @return Inverse of this DynamicTransform.
     */
    DynamicTransform getInverse() const;

    /**
     * Returns the composed transformation of the given transformations.
     * @return DynamicTransformation from frame A to frame C.
     */
    DynamicTransform compose(const DynamicTransform& second) const;

    DynamicTransform projectForward(float dt) const;

    /* Getters */
    inline Position getTranslation() const { return Position(translation); };

    inline Vector getVelocity() const { return Vector(transVel); };

    inline Vector getAcceleration() const { return Vector(transAcc); };

    inline Orientation getRotation() const { return Orientation(rotation); }

    // inline Vector getAngularVel() const { return Vector(angVel); };

    /**
     * Get the roll of this transformation
     */
    float getRoll() const;

    /**
     * Get the pitch of this transformation
     */
    float getPitch() const;

    /**
     * Get the yaw of this transformation
     */
    float getYaw() const;

    /**
     * Get the x-component of this transform's translation
     */
    inline float getX() const { return this->translation.data[0]; }

    /**
     * Get the y-component of this transform's translation
     */
    inline float getY() const { return this->translation.data[1]; }

    /**
     * Get the z-component of this transform's translation
     */
    inline float getZ() const { return this->translation.data[2]; }

private:
    /**
     * Translation vector.
     */
    CMSISMat<3, 1> translation;

    /**
     * Velocity vector.
     */
    CMSISMat<3, 1> transVel;

    /**
     * Acceleration vector.
     */
    CMSISMat<3, 1> transAcc;

    /**
     * Rotation matrix.
     */
    CMSISMat<3, 3> rotation;

    /**
     * Transpose of rotation matrix. Computed and stored at beginning
     * for use in other computations.
     *
     * The transpose of a rotation is its inverse.
     */
    CMSISMat<3, 3> tRotation;

    /**
     * Angular velocity skew matrix.
     */
    CMSISMat<3, 3> angVel;
};  // class DynamicTransform
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_TRANSFORM_HPP_
