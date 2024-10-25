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

#ifndef TAPROOT_WRAPPED_QUANTITY_HPP_
#define TAPROOT_WRAPPED_QUANTITY_HPP_
#include "tap/algorithms/math_user_utils.hpp"

#include "unit_math.hpp"

namespace tap::units
{
/**
 * @brief A quantity that is wrapped between a range of values
 */
template <isQuantity Q>
class Wrapped : public Named<Q>
{
protected:
    using Internal = Named<Q>;
    using Similar = Q::Self<Q::frame>;
    int revolutions{0};
    Similar lower, upper;

    void wrapValue()
    {
        if (this->wrapped >= this->upper)
        {
            this->revolutions++;
            this->value -= this->upper - this->lower;
        }
        else if (this->wrapped < this->lower)
        {
            this->revolutions--;
            this->value += this->upper - this->lower;
        }
    }

public:
    /**
     * @brief Construct a new Wrapped object, with an initial value and bounds
     * @param value the initial value (will be wrapped)
     * @param lower the lower bound
     * @param upper the upper bound
     */
    constexpr Wrapped(Q value, Similar lower, Similar upper)
        : lower(math::min(lower, upper)),
          upper(math::max(lower, upper)),
          Internal(value)
    {
        wrapValue();
    }

    /**
     * @brief Returns the raw/unwrapped value of the quantity
     * @return The value of the quantity
     */
    constexpr float valueOfUnwrapped() const
    {
        return (this->value + this->revolutions * (this->upper - this->lower)).valueOf();
    }

    /**
     * @brief Unwraps the quantity and returns it
     * @return The unwrapped value of the quantity
     */
    constexpr Internal getUnwrapped() const { return Internal(valueOfUnwrapped()); }

    /**
     * @brief Create a new Wrapped object with the same bounds as the current object
     * @param value the value of the new object
     * @param lowerBoundScale a scale factor for the lower bound (defaults to 1 to not scale)
     * @param upperBoundScale a scale factor for the upper bound (defaults to 1 to not scale)
     * @return Wrapped<Internal> the new Wrapped object
     */
    template <int F = Q::frame>
    inline Wrapped<Named<Similar<F>>> withSameBounds(
        const Similar<F> value,
        float lowerBoundScale = 1,
        float upperBoundScale = 1) const
    {
        return Wrapped<Named<Similar<F>>>(
            value,
            this->lower.inOtherFrame<F>(lowerBoundScale),
            this->upper.inOtherFrame<F>(upperBoundScale));
    }

    /**
     * @brief Determine if a value is within the bounds of the Wrapped object
     * @param value the value to check
     */
    inline bool withinRange(const Q& value) const
    {
        return (value > this->lower && value < this->upper);
    }

    /**
     * @brief Asserts that wrapped Isomorphic Quantity has the same bounds as this one
     * @param other The other Wrapped Quantity
     */
    template <isQuantity R>
    constexpr void assertBoundsEqual(const Wrapped<R>& other) requires Isomorphic<Q, R>
    {
        modm_assert(
            compareFloatClose(this->lower.valueOf(), other.lower.valueOf(), 1E-8f),
            "WrappedQuantity::assertBoundsEqual",
            "Lower bounds do not match");
        modm_assert(
            compareFloatClose(this->upper.valueOf(), other.upper.valueOf(), 1E-8f),
            "WrappedQuantity::assertBoundsEqual",
            "Upper bounds do not match");
    }

    /**
     * @brief Get the lower bound of the Wrapped object
     * @return Q the lower bound
     */
    constexpr Q getLowerBound() const { return lower; }

    /**
     * @brief Get the upper bound of the Wrapped object
     * @return Q the upper bound
     */
    constexpr Q getUpperBound() const { return upper; }

    /**
     * @brief Adds another quantity to this one
     * @param other The right hand addend
     * @throws: An assertion error if the other quantity is wrapped, and it has different bounds.
     */
    template <isQuantity R>
    constexpr void operator+=(const R& other) requires IsomorphicFrame<Q, R>
    {
        constexpr if (std::is_base_of<Wrapped, R>) { assertBoundsEqual(other); }
        Internal::operator+=(other);
        this->value = wrap(Internal, lower, upper);
    }

    /**
     * @brief Subtracts another quantity from this one
     * @param other The right hand minuend
     * @throws:  An assertion error if the other quantity is wrapped, and it has different bounds.
     */
    constexpr void operator-=(const Similar& other)
    {
        constexpr if (std::is_base_of<Wrapped, R>) { assertBoundsEqual(other); }
        Internal::operator-=(other);
        this->value = wrap(Internal, lower, upper);
    }

    /**
     * @brief Multiplies this quantity by a unitless factor
     * @param multiple The factor to multiply by
     */
    constexpr void operator*=(const float multiple)
    {
        Internal::operator*=(multiple);
        this->value = wrap(Internal, lower, upper);
    }

    /**
     * Finds the minimum difference against another wrapped value. Can be thought of as the minimum
     * distance between two points on a circle's perimeter.
     *
     * @param[in] other: The wrapped isomorphic quantity to compute the minDifference with.
     * @return: The signed minimum distance.
     * @throws: An assertion error if the two WrappedFloats have different lower and upper bounds.
     */
    template <isQuantity R>
    Internal minDifference(const R& other) requires Isomorphic<Q, R> const
    {
        Wrapped<R::Self> otherWrapped;
        constexpr if (std::is_base_of<Wrapped, R>) { otherWrapped = other; }
        else { otherWrapped = withSameBounds(other); }
        assertBoundsEqual(other);

        Internal interval = this->upper - this->lower;
        Internal difference_between = other.getWrappedValue() - this->getWrappedValue();
        Internal difference_around =
            difference_between + ((difference_between < 0) ? interval : -interval);
        return (abs(difference_between) < abs(difference_around)) ? difference_between
                                                                  : difference_around;
    }
};
}  // namespace tap::units
#endif  // TAPROOT_WRAPPED_QUANTITY_HPP_