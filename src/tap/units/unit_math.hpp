/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_UNIT_MATH_HPP_
#define TAPROOT_UNIT_MATH_HPP_

#include <algorithm>
#include <cmath>

#include "quantity.hpp"
#include "units.hpp"

namespace tap::units::math
{
/**
 * @brief Takes the absolute value of a quantity
 *
 * @param lhs the quantity
 * @return constexpr Q the absolute value
 */
template <isQuantity Q>
constexpr Q abs(const Q& lhs)
{
    return Q(std::abs(lhs.valueOf()));
}

/**
 * @brief Takes the maximum (closest to +infinity) of two isomorphic quantities
 *
 * @param lhs the first operand
 * @param rhs the second operand
 * @return constexpr Q
 */
template <isQuantity Q, isQuantity R>
constexpr Q max(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return (lhs > rhs ? lhs : rhs);
}

/**
 * @brief Takes the minimum (closest to -infinity) of two isomorphic quantites
 *
 * @param lhs the first operand
 * @param rhs the second operand
 * @return constexpr Q
 */
template <isQuantity Q, isQuantity R>
constexpr Q min(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return (lhs < rhs ? lhs : rhs);
}

/**
 * @brief Takes the R'th power of a quantity
 *
 * @tparam R the power to raise the quantity to
 * @param lhs
 * @return constexpr S the result of the operation
 */
template <int R, isQuantity Q, isQuantity S = Exponentiated<Q, ratio<R>>>
constexpr S pow(const Q& lhs)
{
    return S(std::pow(lhs.valueOf(), R));
}

/**
 * @brief Takes the square of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the square of the quantity
 */
template <isQuantity Q, isQuantity S = Exponentiated<Q, ratio<2>>>
constexpr S square(const Q& lhs)
{
    return pow<2>(lhs);
}

/**
 * @brief Takes the cube of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the cube of the quantity
 */
template <isQuantity Q, isQuantity S = Exponentiated<Q, ratio<3>>>
constexpr S cube(const Q& lhs)
{
    return pow<3>(lhs);
}

/**
 * @brief Takes the R'th root of a quantity

 * @tparam R the order of the root
 * @param lhs the quantity
 * @return constexpr S the R root of the quantity
 */
template <int R, isQuantity Q, isQuantity S = Exponentiated<Q, ratio<1, R>>>
constexpr S root(const Q& lhs)
{
    return S(std::pow(lhs.valueOf(), 1.0 / R));
}

/**
 * @brief Takes the square root of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the square root of the quantity
 */
template <isQuantity Q, isQuantity S = Exponentiated<Q, ratio<1, 2>>>
constexpr S sqrt(const Q& lhs)
{
    return root<2>(lhs);
}

/**
 * @brief Takes the cube root of a quantity
 *
 * @param lhs the quantity
 * @return constexpr S the cube root of the quantity
 */
template <isQuantity Q, isQuantity S = Exponentiated<Q, ratio<1, 3>>>
constexpr S cbrt(const Q& lhs)
{
    return root<3>(lhs);
}

/**
 * @brief Calculates the hypotenuse of a right triangle with two sides of isomorphic quantities
 *
 * @param lhs x side
 * @param rhs y side
 * @return constexpr Q the hypotenuse
 */
template <isQuantity Q, isQuantity R>
constexpr Q hypot(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::hypot(lhs.valueOf(), rhs.valueOf()));
}

/**
 * @brief Returns the remainder of a division of two isomorphic quantities
 *
 * @param lhs the dividend
 * @param rhs the divisor
 * @return constexpr Q the remainder
 */
template <isQuantity Q, isQuantity R>
constexpr Q mod(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::fmod(lhs.valueOf(), rhs.valueOf()));
}

/**
 * @brief Returns the absolute value of x with the sign of y
 *
 * @param lhs the quantity to take the absolute value of (x)
 * @param rhs the quantity to take the sign of (y)
 * @return constexpr the first quantity with the sign of the second
 */
template <isQuantity Q1, isQuantity Q2>
constexpr Q1 copysign(const Q1& lhs, const Q2& rhs)
{
    return Q1(std::copysign(lhs.valueOf(), rhs.valueOf()));
}

/**
 * @brief Returns the sign of a quantity
 *
 * @param lhs the quantity
 * @return constexpr int the sign of the quantity
 */
template <isQuantity Q>
constexpr int sign(const Q& lhs)
{
    return lhs.valueOf() < 0 ? -1 : 1;
}

/**
 * @brief Returns true if the quantity is negative
 *
 * @param lhs the quantity
 * @return true if the quantity is negative, false otherwise
 */
template <isQuantity Q>
constexpr bool signbit(const Q& lhs)
{
    return std::signbit(lhs.valueOf());
}

/**
 * @brief Clamps a quantity between two other isomporphic quantities
 *
 * @param lhs the quantity to clamp
 * @param lo the lower bound
 * @param hi the upper bound
 * @return constexpr Q the clamped quantity
 */
template <isQuantity Q, isQuantity R, isQuantity S>
constexpr Q clamp(const Q& lhs, const R& lo, const S& hi) requires Isomorphic<Q, R, S>
{
    return Q(std::clamp(lhs.valueOf(), lo.valueOf(), hi.valueOf()));
}

/**
 * @brief Rounds a quantity up (towards +infinity) to the nearest multiple of another isomorphic
 * quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q ceil(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::ceil(lhs.valueOf() / rhs.valueOf()) * rhs.valueOf());
}

/**
 * @brief Rounds a quantity down (towards -infinity) to the nearest multiple of another isomorphic
 * quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q floor(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::floor(lhs.valueOf() / rhs.valueOf()) * rhs.valueOf());
}

/**
 * @brief Rounds a quantity (towards zero) to the nearest multiple of another isomorphic quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q trunc(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::trunc(lhs.valueOf() / rhs.valueOf()) * rhs.valueOf());
}

/**
 * @brief Rounds a quantity to the nearest multiple of another isomorphic quantity
 * @param lhs the quantity to round
 * @param rhs the multiple to round to
 */
template <isQuantity Q, isQuantity R>
constexpr Q round(const Q& lhs, const R& rhs) requires Isomorphic<Q, R>
{
    return Q(std::round(lhs.valueOf() / rhs.valueOf()) * rhs.valueOf());
}

/**
 * @brief Wraps a quantity between two other isomorphic quantities
 * @param value the quantity to wrap
 * @param lower the lower bound
 * @param upper the upper bound
 */
template <isQuantity Q, isQuantity R, isQuantity S>
constexpr Q wrap(Q value, R lower, S upper) requires Isomorphic<Q, R, S>
{
    if (lower > upper) return wrap(value, upper, lower);
    return (value < Q(0) ? upper : lower) + mod(value, upper - lower);
}

/**
 * @brief Calculates the trigonometric sine of an angle
 * @param rhs the angle
 * @return constexpr Number the sine of the angle
 */
template <int F = 0, int G>
constexpr Number<F> sin(const Angle<G>& rhs)
{
    return Number<F>(std::sin(rhs.valueOf()));
}

/**
 * @brief Calculates the trigonometric cosine of an angle
 * @param rhs the angle
 * @return constexpr Number the cosine of the angle
 */
template <int F = 0, int G>
constexpr Number<F> cos(const Angle<G>& rhs)
{
    return Number<F>(std::cos(rhs.valueOf()));
}

/**
 * @brief Calculates the trigonometric tangent of an angle
 * @param rhs the angle
 * @return constexpr Number the tangent of the angle
 */
template <int F = 0, int G>
constexpr Number<F> tan(const Angle<G>& rhs)
{
    return Number<F>(std::tan(rhs.valueOf()));
}

/**
 * @brief Calculates the trigonometric arcsin of a ratio
 * @param rhs the ratio
 * @return the angle
 */
template <int F = 0, isQuantity Q>
constexpr Angle<F> asin(const Q& rhs)
{
    return Angle<F>(std::asin(rhs.valueOf()));
}

/**
 * @brief Calculates the trigonometric arccosine of a ratio
 * @param rhs the ratio
 * @return the angle
 */
template <int F = 0, isQuantity Q>
constexpr Angle<F> acos(const Q& rhs)
{
    return Angle<F>(std::acos(rhs.valueOf()));
}

/**
 * @brief Calculates the trigonometric arctangent of a angle
 * @param rhs the ratio
 * @return the angle
 */
template <int F = 0, isQuantity Q>
constexpr Angle<F> atan(const Q& rhs)
{
    return Angle<F>(std::atan(rhs.valueOf()));
}

/**
 * @brief Calculates the trigonometric arctangent of a ratio
 * @param lhs the y coordinate
 * @param rhs the x coordinate
 * @return the angle
 */
template <int F = 0, isQuantity Q>
constexpr Angle<F> atan2(const Q& lhs, const Q& rhs)
{
    return Angle<F>(std::atan2(lhs.valueOf(), rhs.valueOf()));
}

/**
 * @brief Compares two quantities for closeness
 * @param lhs the first quantity
 * @param rhs the second quantity
 * @param epsilon the maximum difference between the two quantities
 * @return true if the quantities are within epsilon of each other, false otherwise
 */
template <isQuantity Q, isQuantity R, isQuantity S>
constexpr bool compareClose(
    const Q& lhs,
    const R& rhs,
    const S& epsilon) requires Isomorphic<Q, R, S>
{
    return abs(lhs - rhs) <= epsilon;
}

}  // namespace tap::units::math
#endif