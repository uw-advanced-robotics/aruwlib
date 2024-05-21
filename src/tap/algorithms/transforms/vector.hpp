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

#ifndef TAPROOT_VECTOR_HPP_
#define TAPROOT_VECTOR_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/transforms/position.hpp"

namespace tap::algorithms::transforms
{
// forward declare position to avoid circular dependency
class Position;

class Vector
{
public:
    Vector(float x, float y, float z) : coordinates_({x, y, z}) {}

    Vector(const Vector&& other) : coordinates_(std::move(other.coordinates_)) {}

    Vector(const Vector& other) : coordinates_(CMSISMat(other.coordinates_)) {}

    Vector(CMSISMat<3, 1>& coordinates) : coordinates_(CMSISMat(coordinates)) {}

    Vector(CMSISMat<3, 1>&& coordinates) : coordinates_(std::move(coordinates)) {}

    inline float x() const { return coordinates_.data[0]; }

    inline float y() const { return coordinates_.data[1]; }

    inline float z() const { return coordinates_.data[2]; }

    inline Vector operator+(const Vector& other) const;

    inline Vector operator+(const Position& other) const;

    inline Vector operator*(const float scale) const { return Vector(this->coordinates_ * scale); }

    inline float operator*(const Vector& other) const
    {
        return this->x() * other.x() + this->y() * other.y() + this->z() * other.z();
    }

    inline Vector operator/(const float scale) const { return Vector(this->coordinates_ / scale); }

    const inline CMSISMat<3, 1>& coordinates() const { return coordinates_; }

    const inline float magnitude() const { return sqrt(*this * *this); }

private:
    CMSISMat<3, 1> coordinates_;
};  // class Vector
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_VECTOR_HPP_
