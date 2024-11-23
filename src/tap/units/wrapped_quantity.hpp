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
template <isQuantityOrArithmetic Q, typename F>
using IQInternal = std::conditional_t<std::is_arithmetic_v<Q>, Number<F>, Q>;

template <isQuantityOrArithmetic Q>
struct valueOf
{
    static constexpr float value(const Q &q) { return q; }
};

template <isQuantity Q>
struct valueOf<Q>
{
    static constexpr float value(const Q &q) { return q.valueOf(); }
};

/**
 * @brief A quantity that is wrapped between a range of values
 */
template <isQuantityOrArithmetic Q, typename Frame = DefaultFrame>
class Wrapped : public IQInternal<Q, Frame>
{
protected:
    int revolutions{0};
    Q lower, upper;

    /**
     * @brief Wraps the value of the quantity to be within the bounds. Internal function.
     */
    void wrapValue()
    {
        Named<Q> val(IQInternal<Q, Frame>::value);
        while (val >= this->upper)
        {
            this->revolutions++;
            val -= (this->upper - this->lower);
        }
        while (val < this->lower)
        {
            this->revolutions--;
            val += (this->upper - this->lower);
        }
        Internal::value = IQInternal<Q, Frame>(val).valueOf();
    }

    template <isQuantityOrArithmetic T>
    constexpr static T min(T a, T b)
    {
        return a < b ? a : b;
    }

    template <isQuantityOrArithmetic T>
    constexpr static T max(T a, T b)
    {
        return a > b ? a : b;
    }

    template <isQuantityOrArithmetic T>
    constexpr static T abs(const T &val)
    {
        return val < T(0) ? -val : val;
    }

    constexpr static bool compareClose(Q val1, Q val2, Q epsilon)
    {
        Q diff = val1 - val2;
        return (diff < Q(0) ? (Q(0) - diff) : diff) <= epsilon;
    }

public:
    typedef IQInternal<Q, Frame> Internal;
    typedef Named<Q> Similar;

    /**
     * @brief Construct a new Wrapped object, with an initial value and bounds
     * @param value the initial value (will be wrapped)
     * @param lower the lower bound
     * @param upper the upper bound
     */
    explicit constexpr Wrapped(Similar value, Similar lower, Similar upper)
        : Internal(value),
          lower(min(lower, upper)),
          upper(max(lower, upper))

    {
        wrapValue();
    }

    /**
     * @brief Returns the raw/unwrapped value of the quantity
     * @return The value of the quantity
     */
    constexpr float valueOfUnwrapped() const
    {
        return Internal::value + Internal(upper - lower).valueOf() * this->revolutions;
    }

    /**
     * @brief Unwraps the quantity and returns it
     * @return The unwrapped value of the quantity
     */
    constexpr Similar getUnwrapped() const { return Similar(valueOfUnwrapped()); }

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
        Internal::value = Internal(value).valueOf();
        wrapValue();
    }

    /**
     * @brief Create a new Wrapped object with the same bounds as the current object
     * @param value the value of the new object
     * @param lowerBoundScale a scale factor for the lower bound (defaults to 1 to not scale)
     * @param upperBoundScale a scale factor for the upper bound (defaults to 1 to not scale)
     * @return Wrapped<Internal> the new Wrapped objectclea
     */
    template <isQuantityOrArithmetic R>
    inline Wrapped<Named<R>> withSameBounds(
        const R &nvalue,
        float lowerBoundScale = 1,
        float upperBoundScale = 1) const
        requires(IsomorphicFrame<Internal, IQInternal<R, Frame>> || std::is_convertible_v<Q, R>)
    {
        return Wrapped<Named<R>>(nvalue, R(lower * lowerBoundScale), R(upper * upperBoundScale));
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
    void assertBoundsEqual(const R &other) const requires Isomorphic<Internal, R>
    {
        modm_assert(
            Wrapped::compareClose(this->lower, other.getLowerBound(), Similar(1E-8f)),
            "Wrapped::assertBoundsEqual",
            "Lower bounds do not match");
        modm_assert(
            Wrapped::compareClose(this->upper, other.getUpperBound(), Similar(1E-8f)),
            "Wrapped::assertBoundsEqual",
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
    template <isQuantityOrArithmetic R>
    constexpr void operator+=(const R &other) requires(
        IsomorphicFrame<Internal, R> || std::is_convertible_v<Q, R>)
    {
        if constexpr (isWrappedQuantity<R>)
        {
            this->assertBoundsEqual(other);
        }
        Internal::operator+=(Internal(other));
        wrapValue();
    }

    /**
     * @brief Subtracts another quantity from this one
     * @param other The right hand minuend
     * @throws:  An assertion error if the other quantity is wrapped, and it has different bounds.
     */
    template <isQuantityOrArithmetic R>
    constexpr void operator-=(const R &other) requires(
        IsomorphicFrame<Internal, R> || std::is_convertible_v<Q, R>)
    {
        if constexpr (isWrappedQuantity<R>)
        {
            this->assertBoundsEqual(other);
        }
        Internal::operator-=(Internal(other));
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
     * Finds the minimum difference against another value, which will (or must if it is already) be
     * wrapped with the same bounds as this quantity. Can be thought of as the minimum distance
     * between two points on a circle's perimeter.
     *
     * @param[in] other: The isomorphic quantity to compute the minDifference with.
     * @return: The signed minimum distance.
     */
    template <isQuantityOrArithmetic R>
    Similar minDifference(const R &other) requires(
        IsomorphicFrame<Internal, IQInternal<R, Frame>> || std::is_convertible_v<Q, R>)
    {
        if constexpr (isWrappedQuantity<Q>)
        {
            assertBoundsEqual(other);
        }
        Wrapped<R> s = withSameBounds(other);
        Similar interval = this->upper - this->lower;
        Internal difference_between = Internal(s.valueOf() - Internal::valueOf());
        Internal difference_around = difference_between + ((difference_between.valueOf() < 0)
                                                               ? Internal(interval)
                                                               : Internal(0) - Internal(interval));
        return (abs(difference_between) < abs(difference_around))
                   ? Similar(difference_between.valueOf())
                   : Similar(difference_around.valueOf());
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
template <isWrappedQuantity Q, isQuantityOrArithmetic R>
constexpr Q operator+(const Q &lhs, const R &rhs) requires(
    IsomorphicFrame<typename Q::Internal, R> || std::is_convertible_v<typename Q::Similar, R>)
{
    if constexpr (isWrappedQuantity<R>) lhs.assertBoundsEqual(rhs);
    return lhs.withSameBounds(typename Q::Similar(lhs.valueOf() + valueOf<R>::value(rhs)));
}

/**
 * @brief Subtracts a quantity from a wrapped quantity
 * @param lhs The left hand minuend
 * @param other The right hand minuend
 * @throws: An assertion error if the right hand quantity is wrapped, and it has different bounds.
 * @return The difference of the two quantities, wrapped using the same bounds as the left hand
 * operand.
 */
template <isWrappedQuantity Q, isQuantityOrArithmetic R>
constexpr Q operator-(const Q &lhs, const R &rhs) requires(
    IsomorphicFrame<typename Q::Internal, R> || std::is_convertible_v<typename Q::Similar, R>)
{
    if constexpr (isWrappedQuantity<R>) lhs.assertBoundsEqual(rhs);
    return lhs.withSameBounds(typename Q::Similar(lhs.valueOf() - valueOf<R>::value(rhs)));
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
    return lhs.withSameBounds(typename Q::Similar(lhs.valueOfUnwrapped() * rhs));
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
    return lhs.withSameBounds(typename Q::Similar(lhs.valueOfUnwrapped() / rhs));
}

}  // namespace tap::units
#endif  // TAPROOT_WRAPPED_QUANTITY_HPP_