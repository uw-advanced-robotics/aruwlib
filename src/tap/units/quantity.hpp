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

#ifndef TAPROOT_QUANTITY_HPP_
#define TAPROOT_QUANTITY_HPP_

#include <ratio>
#include <type_traits>

using std::ratio, std::ratio_add, std::ratio_subtract, std::ratio_multiply, std::ratio_divide,
    std::ratio_equal;
namespace tap::units
{
/**
 * @brief A class representing a scalar quantity with unit dimensions.
 */
template <
    typename Time = ratio<0>,
    typename Length = ratio<0>,
    typename Mass = ratio<0>,
    typename Current = ratio<0>,
    typename Temperature = ratio<0>,
    typename Angle = ratio<0>,
    int Frame = 0>
class Quantity
{
protected:
    float value;

public:
    // Convenience labels representing the dimensions, for use in template metaprogramming

    /// The time dimension of the quantity, with a base unit of seconds
    typedef Time time;
    /// The length dimension of the quantity, with a base unit of meters
    typedef Length length;
    /// The mass dimension of the quantity, with a base unit of kilograms
    typedef Mass mass;
    /// The current dimension of the quantity, with a base unit of amperes
    typedef Current current;
    /// The temperature dimension of the quantity, with a base unit of kelvin
    typedef Temperature temperature;
    /// The angle dimension of the quantity, with a base unit of radians
    typedef Angle angle;
    /// The frame of reference of the quantity
    static constexpr int frame = Frame;

    /**
     * @brief convenience label. Represents an isomorphic unit (equal dimensions)
     */
    using Self = Quantity<Time, Length, Mass, Current, Temperature, Angle, Frame>;

    /**
     * @brief Convenience label. Represents an isomorphic unit (equal dimensions) in an arbitrary
     * frame.
     */
    template <int F>
    using SelfOtherFrame = Quantity<Time, Length, Mass, Current, Temperature, Angle, F>;

    // Constructors
    /**
     * @brief Construct a new Quantity object
     * @param value The new value of the quantity, in its base unit
     */
    explicit constexpr Quantity(float value) : value(value) {}

    /**
     * @brief Construct a new Quantity object. Default constructor, initializes value to 0
     */
    explicit constexpr Quantity() : value(0) {}

    /**
     * @brief Construct a new Quantity object
     * @param other The other quantity to copy
     */
    template <int F>
    constexpr Quantity(const SelfOtherFrame<F> other) : value(other.value)
    {
    }

    /**
     * @brief Returns the value of the quantity in its base unit
     * @return The value of the quantity
     */
    constexpr float valueOf() const { return value; }

    /**
     * @brief Returns the value of the quantity converted to another unit
     * @param other The other unit to convert to
     */
    template <int F>
    constexpr float convertTo(const SelfOtherFrame<F> unit) const
    {
        return value / unit.value;
    }

    // Operators

    /**
     * @brief Adds another quantity to this one
     * @param other The right hand addend
     */
    constexpr void operator+=(const SelfOtherFrame<Frame> other) { value += other.value; }

    /**
     * @brief Subtracts another quantity from this one
     * @param other The right hand minuend
     */
    constexpr void operator-=(const SelfOtherFrame<Frame> other) { value -= other.value; }

    /**
     * @brief Multiplies this quantity by a unitless factor
     * @param multiple The factor to multiply by
     */
    constexpr void operator*=(const float multiple) { value *= multiple; }

    /**
     * @brief Divides this quantity by a unitless factor
     * @param scalar The factor to divide by
     */
    constexpr void operator/=(const float dividend) { value /= dividend; }

    template <int F = 0>
    constexpr SelfOtherFrame<F> inOtherFrame(float factor = 1)
    {
        return SelfOtherFrame<F>(value * factor);
    }
};

/**
 * @brief Internal utility function to check if a type is a Quantity. Should not be used directly.
 */
template <
    typename T = ratio<0>,
    typename L = ratio<0>,
    typename M = ratio<0>,
    typename C = ratio<0>,
    typename O = ratio<0>,
    typename A = ratio<0>,
    int F = 0>
constexpr void quantityChecker(Quantity<T, L, M, C, O, A, F> q)
{
}

/**
 * @brief Concept to use in template arguments (EX: template <isQuantity Q>). Requires instances of
 * Q to inherit from Quantity and have matching dimensions
 */
template <typename Q>
concept isQuantity = requires(Q q)
{
    quantityChecker(q);
};

/**
 * @brief Concept to use when determining dimensional equivalence.
 * @tparam Q The first quantity type to compare
 * @tparam R Additional quantity type(s) to compare
 */
template <typename Q, typename... R>
concept Isomorphic = isQuantity<Q> && (isQuantity<R> && ...) &&
                     (std::ratio_equal<typename Q::time, typename R::time>::value && ...) &&
                     (std::ratio_equal<typename Q::length, typename R::length>::value && ...) &&
                     (std::ratio_equal<typename Q::mass, typename R::mass>::value && ...) &&
                     (std::ratio_equal<typename Q::current, typename R::current>::value && ...) &&
                     (std::ratio_equal<typename Q::temperature, typename R::temperature>::value &&
                      ...) &&
                     (std::ratio_equal<typename Q::angle, typename R::angle>::value && ...);
/**
 * @brief Concept to use when determing if quantities are in the same frame of reference.
 * @tparam Q The first quantity type to compare
 * @tparam R Additional quantity type(s) to compare
 */
template <typename Q, typename... R>
concept SameFrame = isQuantity<Q> && (isQuantity<R> && ...) && ((Q::frame == R::frame) && ...);

/**
 * @brief Concept to determine if quantities are isomorphic and in the same frame of reference.
 * @tparam Q The first quantity type to compare
 * @tparam R Additional quantity type(s) to compare
 */
template <typename Q, typename... R>
concept IsomorphicFrame = Isomorphic<Q, R...>&& SameFrame<Q, R...>;

/**
 * @brief Utility struct to look up the named class representation of a quantity type. Should not be
 * used directly.
 */
template <isQuantity Q>
struct lookupName
{
    using Named = Q;
};

/**
 * @brief Helper type to look up the named class representation of a quantity type. Should rarely
 * need to be used directly.
 */
template <isQuantity Q>
using Named = typename lookupName<Q>::Named;

/**
 * @brief Simplifiation of Quantity multiplication. Represents two quantity types in the same frame
 * multiplied, as a named type if it exists.
 * @tparam Q The first multiplcand type
 * @tparam R The second multiplicand type
 */
template <isQuantity Q, isQuantity R>
requires(Q::frame == R::frame) using Multiplied = Named<Quantity<
    ratio_add<typename Q::time, typename R::time>,
    ratio_add<typename Q::length, typename R::length>,
    ratio_add<typename Q::mass, typename R::mass>,
    ratio_add<typename Q::current, typename R::current>,
    ratio_add<typename Q::temperature, typename R::temperature>,
    ratio_add<typename Q::angle, typename R::angle>,
    Q::frame>>;

/**
 * @brief Simplifiation of Quantity division. Represents two quantity types in the same frame
 * divided, as a named type if it exists.
 * @tparam Q The dividend type
 * @tparam R The divisor type
 */
template <isQuantity Q, isQuantity R>
requires(Q::frame == R::frame) using Divided = Named<Quantity<
    ratio_subtract<typename Q::time, typename R::time>,
    ratio_subtract<typename Q::length, typename R::length>,
    ratio_subtract<typename Q::mass, typename R::mass>,
    ratio_subtract<typename Q::current, typename R::current>,
    ratio_subtract<typename Q::temperature, typename R::temperature>,
    ratio_subtract<typename Q::angle, typename R::angle>,
    Q::frame>>;

/**
 * @brief Simplifiation of Quantity exponentiation. Represents a quantity type raised to a power, as
 * a named type if it exists.
 * @tparam Q The base quantity type
 * @tparam R The power to raise to
 */
template <isQuantity Q, typename R>
using Exponentiated = Named<Quantity<
    ratio_multiply<typename Q::time, R>,
    ratio_multiply<typename Q::length, R>,
    ratio_multiply<typename Q::mass, R>,
    ratio_multiply<typename Q::current, R>,
    ratio_multiply<typename Q::temperature, R>,
    ratio_multiply<typename Q::angle, R>,
    Q::frame>>;

template <isQuantity Q>
class Wrapped;

/**
 * @brief Internal utility function to check if a type is a Wrapped quantity. Should not be used
 * directly.
 */
template <isQuantity Q>
void WrappedChecker(Wrapped<Q> w)
{
}

/**
 * @brief Concept to use in template arguments (EX: template <isWrappedQuantity Q>). Requires
 * instances of Q to inherit from Wrapped and have matching dimensions
 */
template <typename Q>
concept isWrappedQuantity = requires(Q w)
{
    WrappedChecker(w);
};

/**
 * @brief Adds two isomorphic and same-frame quantities.
 * @param lhs The left hand addend
 * @param rhs The right hand addend
 * @return The sum of the two quantities, as a named type if it exists.
 */
template <isQuantity Q, isQuantity R>
constexpr Named<Q> operator+(Q lhs, R rhs) requires(IsomorphicFrame<Q, R> && !isWrappedQuantity<Q>)
{
    return Named<Q>(lhs.valueOf() + rhs.valueOf());
}

/**
 * @brief Subtracts two isomorphic and same-frame quantities.
 * @param lhs The left hand minuend
 * @param rhs The right hand subtrahend
 * @return The difference of the two quantities, as a named type if it exists.
 */
template <isQuantity Q, isQuantity R>
constexpr Named<Q> operator-(Q lhs, R rhs) requires(IsomorphicFrame<Q, R> && !isWrappedQuantity<Q>)
{
    return Named<Q>(lhs.valueOf() - rhs.valueOf());
}

/**
 * @brief Multiplies a quantity by a unitless scalar.
 * @param lhs The quantity to multiply
 * @param rhs The scalar to multiply by
 * @return The product of the quantity and the scalar, as a named type if it exists.
 */
template <isQuantity Q>
constexpr Named<Q> operator*(Q lhs, float rhs) requires(!isWrappedQuantity<Q>)
{
    return Named<Q>(lhs.valueOf() * rhs);
}

/**
 * @brief Multiplies a quantity by a unitless scalar.
 * @param lhs The scalar to multiply by
 * @param rhs The quantity to multiply
 * @return The product of the quantity and the scalar, as a named type if it exists.
 */
template <isQuantity Q>
constexpr Named<Q> operator*(float lhs, Q rhs)
{
    return Named<Q>(lhs * rhs.valueOf());
}

/**
 * @brief Multiplies two quantities which are in the same reference frame.
 * @param lhs The left hand multiplicand
 * @param rhs The right hand multiplicand
 * @return The product of the two quantities, as a named type if it exists.
 */
template <isQuantity Q, isQuantity R, isQuantity S = Multiplied<Q, R>>
constexpr S operator*(Q lhs, R rhs) requires SameFrame<Q, R>
{
    return S(lhs.valueOf() * rhs.valueOf());
}

/**
 * Divides a quantity by a unitless scalar.
 * @param lhs The quantity to divide
 * @param rhs The scalar to divide by
 * @return The quotient of the quantity and the scalar, as a named type if it exists.
 */
template <isQuantity Q>
constexpr Named<Q> operator/(Q lhs, float rhs) requires(!isWrappedQuantity<Q>)
{
    return Named<Q>(lhs.valueOf() / rhs);
}

/**
 * @brief Divides two quantities which are in the same reference frame.
 * @param lhs The dividend
 * @param rhs The divisor
 * @return The quotient of the two quantities, as a named type if it exists.
 */
template <isQuantity Q, isQuantity R, isQuantity S = Divided<Q, R>>
constexpr S operator/(Q lhs, R rhs) requires SameFrame<Q, R>
{
    return S(lhs.valueOf() / rhs.valueOf());
}

/**
 * @brief Compares two isomorphic quantities for equality.
 * @param lhs The left hand quantity
 * @param rhs The right hand quantity
 * @return True if the two quantities; values are equal, false otherwise.
 */
template <isQuantity Q, isQuantity R>
constexpr bool operator==(Q lhs, R rhs) requires Isomorphic<Q, R>
{
    return lhs.valueOf() == rhs.valueOf();
}

/**
 * @brief Compares two isomorphic quantities for inequality.
 * @param lhs The left hand quantity
 * @param rhs The right hand quantity
 * @return True if the two quantities; values are not equal, false otherwise.
 */
template <isQuantity Q, isQuantity R>
constexpr bool operator!=(Q lhs, R rhs) requires Isomorphic<Q, R>
{
    return lhs.valueOf() != rhs.valueOf();
}

/**
 * @brief Compares two isomorphic quantities for equality.
 * @param lhs The left hand quantity
 * @param rhs The right hand quantity
 * @return True if the left hand quantity is less than the right hand quantity, false otherwise.
 */
template <isQuantity Q, isQuantity R>
constexpr bool operator<(Q lhs, R rhs) requires Isomorphic<Q, R>
{
    return lhs.valueOf() < rhs.valueOf();
}

/**
 * @brief Compares two isomorphic quantities for equality.
 * @param lhs The left hand quantity
 * @param rhs The right hand quantity
 * @return True if the left hand quantity is greater than the right hand quantity, false otherwise.
 */
template <isQuantity Q, isQuantity R>
constexpr bool operator>(Q lhs, R rhs) requires Isomorphic<Q, R>
{
    return lhs.valueOf() > rhs.valueOf();
}

/**
 * @brief Compares two isomorphic quantities for equality.
 * @param lhs The left hand quantity
 * @param rhs The right hand quantity
 * @return True if the left hand quantity is less than or equal to the right hand quantity, false
 * otherwise.
 */
template <isQuantity Q, isQuantity R>
constexpr bool operator<=(Q lhs, R rhs) requires Isomorphic<Q, R>
{
    return lhs.valueOf() <= rhs.valueOf();
}

/**
 * @brief Compares two isomorphic quantities for equality.
 * @param lhs The left hand quantity
 * @param rhs The right hand quantity
 * @return True if the left hand quantity is greater than or equal to the right hand quantity, false
 * otherwise.
 */
template <isQuantity Q, isQuantity R>
constexpr bool operator>=(Q lhs, R rhs) requires Isomorphic<Q, R>
{
    return lhs.valueOf() >= rhs.valueOf();
}
}  // namespace tap::units
#endif  // TAPROOT_QUANTITY_HPP_