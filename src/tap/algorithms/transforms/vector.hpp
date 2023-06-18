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
#include "position.hpp"

namespace tap::algorithms::transforms
{

// Forward declaration for transform.hpp
template <const Frame& SOURCE, const Frame& TARGET>
class Transform;

// Forward declaration for inertial_transform.hpp
template <const Frame& SOURCE, const Frame& TARGET>
class InertialTransform;

template<const Frame& FRAME>
class Vector
{
    template <const Frame& SOURCE, const Frame& TARGET> friend class Transform;
    template <const Frame& SOURCE, const Frame& TARGET> friend class InertialTransform;
public:
    Vector(float x, float y, float z)
        : coordinates_({x, y, z})
    {
    }

    Vector(CMSISMat<3, 1> coordinates)
        : coordinates_(std::move(coordinates))
    {
    }

    // TODO: sort out copy constructor and copy assignment because default directly copies cmsismat

    inline float x() const { return coordinates_.data[0]; }

    inline float y() const { return coordinates_.data[1]; }

    inline float z() const { return coordinates_.data[2]; }

    inline Position<FRAME> operator+(const Position<FRAME>& position) const { return Position<FRAME>(this->coordinates_ + position.coordinates_); }

    inline Vector<FRAME> operator+(const Vector<FRAME>& other) const { return Vector<FRAME>(this->coordinates_ + other.coordinates_); }

    inline Vector<FRAME> operator*(const float scale) const { return Vector<FRAME>(this->coordinates_ * scale); }

    inline Vector<FRAME> operator/(const float scale) const { return Vector<FRAME>(this->coordinates_ / scale); }

    const inline CMSISMat<3, 1>& coordinates() const { return coordinates_; }

private:
    CMSISMat<3, 1> coordinates_;
};  // class Vector
}   // namespace tap::algorithms::transforms

#endif  // TAPROOT_VECTOR_HPP_
