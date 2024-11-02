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
NEW_UNIT(Temperature, KELVIN, Kelvin, K, 0, 0, 0, 0, 1, 0)
NEW_UNIT_LITERAL(Temperature, RANKINE, Rankine, R, KELVIN<F> / 1.8f)

namespace constants
{
template <int F = 0>
constexpr Temperature FAHRENHEIT = RANKINE<F>;
template <int F = 0>
constexpr Temperature CELSIUS = KELVIN<F>;

template <int F = 0>
constexpr Temperature WATER_FREEZING_POINT = Temperature<F>(273.15f);
template <int F = 0>
constexpr Temperature WATER_BOILING_POINT = Temperature<F>(373.15f);
}  // namespace constants

namespace conversions
{
template <int F = 0>
constexpr inline Temperature<F> fromFahrenheit(float value)
{
    return Temperature<F>((value - 32) * (5.0 / 9.0) + 273.15);
}
template <int F = 0>
constexpr inline float toFahrenheit(Temperature<F> quantity)
{
    return (quantity.valueOf() - 273.15f) * (9.0 / 5.0) + 32;
}
template <int F = 0>
constexpr inline Temperature<F> fromCelsius(float value)
{
    return Temperature(value + 273.15f);
}
template <int F = 0>
constexpr inline float toCelsius(Temperature<F> quantity)
{
    return quantity.valueOf() - 273.15f;
}
}  // namespace conversions
namespace literals
{
constexpr Temperature<> operator"" _degF(long double value)
{
    return conversions::fromFahrenheit(static_cast<float>(value));
}
constexpr Temperature<> operator"" _degF(unsigned long long value)
{
    return conversions::fromFahrenheit(static_cast<float>(value));
}
constexpr Temperature<> operator"" _degC(long double value)
{
    return conversions::fromCelsius(static_cast<float>(value));
}
constexpr Temperature<> operator"" _degC(unsigned long long value)
{
    return conversions::fromCelsius(static_cast<float>(value));
}
}  // namespace literals
}  // namespace tap::units
#endif