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

#include "dynamic_transform.hpp"

namespace tap::algorithms::transforms
{
// DynamicTransform::DynamicTransform(
//     const Position& translation,
//     const Orientation& rotation,
//     const Vector& velocity,
//     const Vector& acceleration,
//     const Vector& angularVelocity)
//     : translation(translation.coordinates()),
//       transVel(velocity.coordinates()),
//       transAcc(acceleration.coordinates()),
//       rotation(rotation.matrix()),
//       tRotation(rotation.matrix().transpose()),
//       angVel(angularVelocity.coordinates())
// {
// }

// DynamicTransform::DynamicTransform(
//     Position&& translation,
//     Orientation&& rotation,
//     Vector&& velocity,
//     Vector&& acceleration,
//     Vector&& angularVelocity)
//     : translation(std::move(translation.coordinates())),
//       transVel(std::move(velocity.coordinates())),
//       transAcc(std::move(acceleration.coordinates())),
//       rotation(std::move(rotation.matrix())),
//       tRotation(rotation.matrix().transpose()),
//       angVel(std::move(angularVelocity.coordinates()))
// {
// }

DynamicTransform::DynamicTransform(
    const CMSISMat<3, 1>& translation,
    const CMSISMat<3, 3>& rotation,
    const CMSISMat<3, 1>& velocity,
    const CMSISMat<3, 1>& acceleration,
    const CMSISMat<3, 3>& angularVelocity)
    : translation(translation),
      transVel(velocity),
      transAcc(acceleration),
      rotation(rotation),
      tRotation(rotation.transpose()),
      angVel(angularVelocity)
{
}

DynamicTransform::DynamicTransform(
    CMSISMat<3, 1>&& translation,
    CMSISMat<3, 3>&& rotation,
    CMSISMat<3, 1>&& velocity,
    CMSISMat<3, 1>&& acceleration,
    CMSISMat<3, 3>&& angularVelocity)
    : translation(std::move(translation)),
      transVel(std::move(velocity)),
      transAcc(std::move(acceleration)),
      rotation(std::move(rotation)),
      tRotation(rotation.transpose()),
      angVel(std::move(angularVelocity))
{
}

DynamicTransform::DynamicTransform(
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
    float yawVel)
    : translation({x, y, z}),
      transVel({vx, vy, vz}),
      transAcc({ax, ay, az}),
      rotation(fromEulerAngles(roll, pitch, yaw)),
      angVel(skewMatFromAngVel(rollVel, pitchVel, yawVel)),
      tRotation(rotation.transpose())
{
}

Position DynamicTransform::apply(const Position& position) const
{
    return Position(tRotation * (position.coordinates() - translation));
}

Vector DynamicTransform::apply(const Vector& vector) const
{
    return Vector(tRotation * vector.coordinates());
}

Orientation DynamicTransform::apply(const Orientation& orientation) const
{
    return Orientation(tRotation * orientation.matrix());
}

DynamicTransform DynamicTransform::getInverse() const
{
    // negative transposed rotation matrix times original position = new position
    CMSISMat<3, 1> invTranslation = tRotation * translation;
    invTranslation = -invTranslation;
    return DynamicTransform(invTranslation, tRotation, -transVel, -transAcc, -angVel);
}

DynamicTransform DynamicTransform::compose(const DynamicTransform& second) const
{
    CMSISMat<3, 3> newRot = this->rotation * second.rotation;
    CMSISMat<3, 1> newPos = this->translation + this->rotation * second.translation;
    CMSISMat<3, 1> newVel = this->transVel + this->angVel * this->rotation * second.translation +
                            this->rotation * second.transVel;
    CMSISMat<3, 1> newAcc =
        this->transAcc + this->angVel * this->angVel * this->rotation * second.translation +
        2 * this->angVel * this->rotation * second.transVel + this->rotation * second.transAcc;
    CMSISMat<3, 3> newAngVel = this->angVel + this->rotation * second.angVel * this->tRotation;
    return DynamicTransform(newPos, newRot, newVel, newAcc, newAngVel);
}

DynamicTransform DynamicTransform::projectForward(float dt) const
{
    CMSISMat<3, 3> velDt = CMSISMat<3, 3>();
    velDt.constructIdentityMatrix();
    velDt += sin(dt) * this->angVel + (1 - cos(dt)) * this->angVel * this->angVel;
    CMSISMat<3, 3> newRot = velDt * this->rotation;
    CMSISMat<3, 1> newPos = this->translation;  // I HAVE NO FUCKING CLUE RN
    CMSISMat<3, 1> newVel = velDt * (this->transVel + this->transAcc * dt);  // IDK
    return DynamicTransform(newPos, newRot, newVel, this->transAcc, this->angVel);
}

float DynamicTransform::getRoll() const
{
    float jz = rotation.data[2 * 3 + 1];
    float kz = rotation.data[2 * 3 + 2];
    return atan2(jz, kz);
}

float DynamicTransform::getPitch() const
{
    float iz = rotation.data[2 * 3 + 0];
    return asinf(-iz);
}

float DynamicTransform::getYaw() const
{
    float iy = rotation.data[1 * 3 + 0];
    float ix = rotation.data[0 * 3 + 0];
    return atan2(iy, ix);
}

}  // namespace tap::algorithms::transforms
