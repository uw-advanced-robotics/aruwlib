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

#include "../algorithms/math_user_utils.hpp"

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
    using Similar = Q::Self;
    int revolutions{0};
    Similar lower, upper;

    /**
     * @brief Wraps the value of the quantity to be within the bounds. Internal function.
     */
    void wrapValue()
    {
        while (Internal::value >= this->upper.valueOf())
        {
            this->revolutions++;
            this->value -= (this->upper - this->lower).valueOf();
        }
        while (Internal::value < this->lower.valueOf())
        {
            this->revolutions--;
            this->value += (this->upper - this->lower).valueOf();
        }
    }

public:
    /**
     * @brief Construct a new Wrapped object, with an initial value and bounds
     * @param value the initial value (will be wrapped)
     * @param lower the lower bound
     * @param upper the upper bound
     */
    explicit constexpr Wrapped(Q value, Similar lower, Similar upper)
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
        return Internal::value + (this->upper - this->lower).valueOf() * this->revolutions;
    }

    /**
     * @brief Unwraps the quantity and returns it
     * @return The unwrapped value of the quantity
     */
    constexpr Internal getUnwrapped() const { return Internal(valueOfUnwrapped()); }

    /**
     * @brief Returns the number of revolutions the quantity has undergone
     * @return The number of revolutions
     */
    constexpr int getRevolutions() const { return revolutions; }

    /**
     * @brief Sets the value of the quantity
     * @param value the new value, which will be wrapped
     */
    void set(Similar value)
    {
        Internal::value = value.valueOf();
        wrapValue();
    }

    /**
     * @brief Create a new Wrapped object with the same bounds as the current object
     * @param value the value of the new object
     * @param lowerBoundScale a scale factor for the lower bound (defaults to 1 to not scale)
     * @param upperBoundScale a scale factor for the upper bound (defaults to 1 to not scale)
     * @return Wrapped<Internal> the new Wrapped object
     */
    template <isQuantity R>
    inline Wrapped<Named<typename R::Self>> withSameBounds(
        const R nvalue,
        float lowerBoundScale = 1,
        float upperBoundScale = 1) const requires Isomorphic<Q, R>
    {
        return Wrapped<Named<typename R::Self>>(
            nvalue,
            Named<typename R::Self>(lower.valueOf() * lowerBoundScale),
            Named<typename R::Self>(upper.valueOf() * upperBoundScale));
    }

    /**
     * @brief Determine if a value is within the bounds of the Wrapped object, Specifically, it
     * checks the bounds [lower, upper)
     * @param value the value to check
     */
    inline bool withinRange(const Similar &value) const
    {
        return (value >= this->lower && value < this->upper);
    }

    /**
     * @brief Asserts that wrapped Isomorphic Quantity has the same bounds as this one
     * @param other The other Wrapped Quantity
     */
    template <isWrappedQuantity R>
    void assertBoundsEqual(const R &other) const requires Isomorphic<Q, R>
    {
        modm_assert(
            tap::algorithms::compareFloatClose(
                this->lower.valueOf(),
                other.getLowerBound().valueOf(),
                1E-8f),
            "WrappedQuantity::assertBoundsEqual",
            "Lower bounds do not match");
        modm_assert(
            tap::algorithms::compareFloatClose(
                this->upper.valueOf(),
                other.getUpperBound().valueOf(),
                1E-8f),
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
    constexpr void operator+=(const R &other) requires IsomorphicFrame<Q, R>
    {
        if constexpr (isWrappedQuantity<R>)
        {
            this->assertBoundsEqual(other);
        }
        Internal::operator+=(other);
        wrapValue();
    }

    /**
     * @brief Subtracts another quantity from this one
     * @param other The right hand minuend
     * @throws:  An assertion error if the other quantity is wrapped, and it has different bounds.
     */
    template <isQuantity R>
    constexpr void operator-=(const R &other) requires IsomorphicFrame<Q, R>
    {
        if constexpr (isWrappedQuantity<R>)
        {
            this->assertBoundsEqual(other);
        }
        Internal::operator-=(other);
        wrapValue();
    }

    /**
     * @brief Multiplies this quantity by a unitless factor
     * @param multiple The factor to multiply by
     */
    constexpr void operator*=(const float multiple)
    {
        Internal::operator*=(multiple);
        wrapValue();
    }

    /**
     * Finds the minimum difference against another value, which will be wrapped with the same
     * bounds as this quantity. Can be thought of as the minimum distance between two points on a
     * circle's perimeter.
     *
     * @param[in] other: The isomorphic quantity to compute the minDifference with.
     * @return: The signed minimum distance.
     */
    template <isQuantity R>
    Internal minDifference(const R &other) requires (Isomorphic<Q, R> && !isWrappedQuantity<R>)
    {
        Wrapped<typename R::Self> s(other, this->lower, this->upper);
        Internal interval = this->upper - this->lower;
        Internal difference_between = Internal(s.valueOf() - Internal::valueOf());
        Internal difference_around =
            difference_between +
            ((difference_between.valueOf() < 0) ? interval : Internal(0) - interval);
        return (math::abs(difference_between) < math::abs(difference_around)) ? difference_between
                                                                              : difference_around;
    }

    /**
     * Finds the minimum difference against another wrapped value. Can be thought of as the minimum
     * distance between two points on a circle's perimeter.
     *
     * @param[in] other: The wrapped isomorphic quantity to compute the minDifference with.
     * @return: The signed minimum distance.
     * @throws: An assertion error if the two values have different lower and upper bounds.
     */
    template <isWrappedQuantity R>
    Internal minDifference(const R &other) requires Isomorphic<Q, R>
    {
        assertBoundsEqual(other);
        Internal interval = this->upper - this->lower;
        Internal difference_between = Internal(other.valueOf() - Internal::valueOf());
        Internal difference_around =
            difference_between +
            ((difference_between.valueOf() < 0) ? interval : Internal(0) - interval);
        return (math::abs(difference_between) < math::abs(difference_around)) ? difference_between
                                                                              : difference_around;
    }

    /**
     * Shifts both bounds by the specified amount.
     *
     * @param[in] shiftMagnitude the amount to add to each bound.
     */
    void shiftBounds(const Similar shiftMagnitude)
    {
        upper += shiftMagnitude;
        lower += shiftMagnitude;
        wrapValue();
    }
};

/**
 * @brief Adds a quantity to a wrapped quantity
 * @param lhs The left hand addend
 * @param other The right hand addend
 * @throws: An assertion error if the right hand quantity is wrapped, and it has different bounds.
 * @return The sum of the two quantities, wrapped using the same bounds as the left hand operand.
 */
template <isWrappedQuantity Q, isQuantity R>
constexpr Q operator+(const Q &lhs, const R &rhs) requires IsomorphicFrame<Q, R>
{
    if constexpr (isWrappedQuantity<R>) lhs.assertBoundsEqual(rhs);
    return Q(
        typename R::Self(lhs.valueOf() + rhs.valueOf()),
        lhs.getLowerBound(),
        lhs.getUpperBound());
}

/**
 * @brief Subtracts a quantity from  a wrapped quantity
 * @param lhs The left hand minuend
 * @param other The right hand minuend
 * @throws: An assertion error if the right hand quantity is wrapped, and it has different bounds.
 * @return The difference of the two quantities, wrapped using the same bounds as the left hand
 * operand.
 */
template <isWrappedQuantity Q, isQuantity R>
constexpr Q operator-(const Q &lhs, const R &rhs) requires IsomorphicFrame<Q, R>
{
    if constexpr (isWrappedQuantity<R>) lhs.assertBoundsEqual(rhs);
    return Q(
        typename R::Self(typename R::Self(lhs.valueOf() - rhs.valueOf())),
        lhs.getLowerBound(),
        lhs.getUpperBound());
}

/**
 * @brief Multiplies a wrapped quantity by a unitless factor
 * @param lhs The quantity to multiply
 * @param multiple The factor to multiply by
 * @return The product of the quantity and the factor, wrapped using the bounds of the left hand
 * operand.
 */
template <isWrappedQuantity Q>
constexpr Q operator*(const Q &lhs, float rhs)
{
    return Q(typename Q::Self(lhs.valueOf() * rhs), lhs.getLowerBound(), lhs.getUpperBound());
}

/**
 * @brief Divides a wrapped quantity by a unitless factor
 * @param lhs The quantity to divide
 * @param multiple The factor to divide  by
 * @return The quotient of the quantity and the factor, wrapped using the bounds of the left hand
 * operand.
 */
template <isWrappedQuantity Q>
constexpr Q operator/(const Q &lhs, float rhs)
{
    return Q(typename Q::Self(lhs.valueOf() / rhs), lhs.getLowerBound(), lhs.getUpperBound());
}

}  // namespace tap::units
#endif  // TAPROOT_WRAPPED_QUANTITY_HPP_