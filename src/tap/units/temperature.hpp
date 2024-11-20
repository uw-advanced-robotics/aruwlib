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

#ifndef TAPROOT_TEMPERATURE_HPP_
#define TAPROOT_TEMPERATURE_HPP_
#include "unit_macros.hpp"
namespace tap::units
{
// Temperature
NEW_UNIT(Temperature, DEGREE_CELSIUS, DegreesCelsius, degC, 0, 0, 0, 0, 1, 0)

namespace constants
{
template <typename F = DefaultFrame>
constexpr Temperature DEGREE_FAHRENHEIT = DEGREE_CELSIUS<F> / 1.8f;
template <typename F = DefaultFrame>
constexpr Temperature KELVIN = DEGREE_CELSIUS<F>;
template <typename F = DefaultFrame>
constexpr Temperature WATER_FREEZING_POINT = Temperature<F>(0.0f);
template <typename F = DefaultFrame>
constexpr Temperature WATER_BOILING_POINT = Temperature<F>(100.0f);
}  // namespace constants

namespace conversions
{
template <typename F = DefaultFrame>
constexpr inline Temperature<F> fromDegreesFahrenheit(float value)
{
    return Temperature((value - 32.0f) * (5.0f / 9.0f));
}
template <typename F = DefaultFrame>
constexpr inline Temperature<F> fromKelvin(float value)
{
    return Temperature(value - 273.15f);
}
template <typename F = DefaultFrame>
constexpr inline float toDegreesFahrenheit(Temperature<F> quantity)
{
    return (quantity.valueOf() * 1.8f) + 32.0f;
}
template <typename F = DefaultFrame>
constexpr inline float toKelvin(Temperature<F> quantity)
{
    return quantity.valueOf() + 273.15f;
}
}  // namespace conversions
namespace literals
{
constexpr Temperature<> operator"" _degF(long double value)
{
    return conversions::fromDegreesFahrenheit(static_cast<float>(value));
}
constexpr Temperature<> operator"" _degF(unsigned long long value)
{
    return conversions::fromDegreesFahrenheit(static_cast<float>(value));
}
constexpr Temperature<> operator"" _kelvin(long double value)
{
    return conversions::fromKelvin(static_cast<float>(value));
}
constexpr Temperature<> operator"" _kelvin(unsigned long long value)
{
    return conversions::fromKelvin(static_cast<float>(value));
}
}  // namespace literals
}  // namespace tap::units
#endif