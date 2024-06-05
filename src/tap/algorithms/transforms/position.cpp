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

#include "position.hpp"

#include "vector.hpp"

namespace tap::algorithms::transforms
{
inline Vector Position::operator-(const Vector& other) const
{
    return Vector(this->coordinates_ - other.coordinates());
}

inline Vector Position::operator-(const Position& other) const
{
    return Vector(this->coordinates_ - other.coordinates());
}

inline Position Position::operator+(const Position& vector) const
{
    return Position(this->coordinates_ + vector.coordinates_);
}

inline Position Position::operator*(const float scalar) const
{
    return Position(this->coordinates_ * scalar);
}

Position& Position::operator=(const Position& other)
{
    this->coordinates_ = other.coordinates_;
    return *this;
}

bool Position::operator==(const Position& other) const
{
    return this->coordinates_.data == other.coordinates_.data;
}

float Position::distance(const Position& a, const Position& b) { return (b - a).magnitude(); }

}  // namespace tap::algorithms::transforms
