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

#ifndef TAPROOT_UNITS_HPP_
#define TAPROOT_UNITS_HPP_
#include "temperature.hpp"
#include "unit_macros.hpp"
#include "modm/math/geometry/vector.hpp"

namespace tap::units
{
#define M_PI_F 3.14159265358979323846f
#define M_2PI_F 6.28318530717958647692f

// Number
NEW_UNIT(Number, NUMBER, Number, n, 0, 0, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Number, PERCENT, Percent, pct, NUMBER<F> * 0.01f);

// Time, Frequency
NEW_UNIT(Time, SECOND, Second, s, 1, 0, 0, 0, 0, 0)
UNIT_METRIC_PREFIXES_SMALL(Time, SECOND, second, s)
NEW_UNIT_LITERAL(Time, MINUTE, Minute, min, SECOND<F> * 60.0f)
NEW_UNIT_LITERAL(Time, HOUR, Hour, hr, MINUTE<F> * 60.0f)
NEW_UNIT_LITERAL(Time, DAY, Day, day, HOUR<F> * 24.0f)

NEW_UNIT(Frequency, HERTZ, Hertz, Hz, -1, 0, 0, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Frequency, HERTZ, hertz, Hz)

// Length, Area, Volume
NEW_UNIT(Length, METER, Meter, m, 0, 1, 0, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Length, METER, meter, m)
NEW_UNIT_LITERAL(Length, INCH, Inch, in, CENTIMETER<F> * 2.54f)
NEW_UNIT_LITERAL(Length, FOOT, Foot, ft, INCH<F> * 12.0f)
NEW_UNIT_LITERAL(Length, YARD, Yard, yd, FOOT<F> * 3.0f)
NEW_UNIT_LITERAL(Length, MILE, Mile, mi, FOOT<F> * 5280.0f)

NEW_UNIT(Area, SQUARE_METER, SquareMeter, m2, 0, 2, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Area, SQUARE_INCH, SquareInch, in2, INCH<F>* INCH<F>)
NEW_UNIT_LITERAL(Area, SQUARE_FOOT, SquareFoot, ft2, FOOT<F>* FOOT<F>)

NEW_UNIT(Volume, CUBIC_METER, CubicMeter, m3, 0, 3, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Volume, CUBIC_INCH, CubicInch, in3, INCH<F>* INCH<F>* INCH<F>)
NEW_UNIT_LITERAL(Volume, LITER, Liter, Li, CUBIC_METER<F> * 0.001f)

// Mass, Inertia
NEW_UNIT(Mass, KILOGRAM, Kilogram, kg, 0, 0, 1, 0, 0, 0)
NEW_UNIT_LITERAL(Mass, GRAM, Gram, g, KILOGRAM<F> * 1E-3f)
UNIT_METRIC_PREFIXES_SMALL(Mass, GRAM, gram, g)
NEW_UNIT_LITERAL(Mass, POUND, Pound, lb, GRAM<F> * 453.59237f)

NEW_UNIT(Inertia, KILOGRAM_METER_SQUARED, KilogramMeterSquared, kgm2, 0, 2, 1, 0, 0, 0)

// Current, Charge, Voltage
NEW_UNIT(Current, AMPERE, Ampere, A, 0, 0, 0, 1, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Current, AMPERE, amp, A)

NEW_UNIT(Charge, COULOMB, Coulomb, Cl, 1, 0, 0, 1, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Charge, COULOMB, coulomb, C)

NEW_UNIT(Voltage, VOLT, Volt, V, -3, 2, 1, -1, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Voltage, VOLT, volt, V)

// Angle
NEW_UNIT(Angle, RADIAN, Radian, rad, 0, 0, 0, 0, 0, 1)
NEW_UNIT_LITERAL(Angle, DEGREE, Degree, deg, RADIAN<F> * 180.0f / M_PI_F)
NEW_UNIT_LITERAL(Angle, ROTATION, Rotation, rot, RADIAN<F>* M_2PI_F)
NEW_UNIT_LITERAL(Angle, ARC_MINUTE, ArcMinute, arcmin, DEGREE<F> / 60.0f)
NEW_UNIT_LITERAL(Angle, ARC_SECOND, ArcSecond, arcsec, DEGREE<F> / 3600.0f)

// Linear Velocity, Acceleration, Jerk
NEW_UNIT(LinearVelocity, METER_PER_SECOND, MetersPerSecond, mps, -1, 1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(LinearVelocity, FOOT_PER_SECOND, FeetPerSecond, fps, FOOT<F> / SECOND<F>)
NEW_UNIT_LITERAL(LinearVelocity, MILE_PER_HOUR, MilesPerHour, mph, MILE<F> / HOUR<F>)
NEW_UNIT_LITERAL(
    LinearVelocity,
    KILOMETERS_PER_HOUR,
    KilometerPerHour,
    kmph,
    KILOMETER<F> / HOUR<F>)

NEW_UNIT(
    LinearAcceleration,
    METER_PER_SECOND_SQUARED,
    MetersPerSecondSquared,
    mps2,
    -2,
    1,
    0,
    0,
    0,
    0)
NEW_UNIT_LITERAL(
    LinearAcceleration,
    FOOT_PER_SECOND_SQUARED,
    FeetPerSecondSquared,
    fps2,
    FOOT<F> / (SECOND<F> * SECOND<F>))

NEW_UNIT(LinearJerk, METER_PER_SECOND_CUBED, MetersPerSecondCubed, mps3, -3, 1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(
    LinearJerk,
    FOOT_PER_SECOND_CUBED,
    FeetPerSecondCubed,
    fps3,
    FOOT<F> / (SECOND<F> * SECOND<F> * SECOND<F>))

// Angular Velocity, Acceleration, Jerk
NEW_UNIT(AngularVelocity, RADIAN_PER_SECOND, RadsPerSecond, radps, -1, 0, 0, 0, 0, 1)
NEW_UNIT_LITERAL(AngularVelocity, RPM, Rpm, rpm, ROTATION<F> / MINUTE<F>)

NEW_UNIT(
    AngularAcceleration,
    RADIAN_PER_SECOND_SQUARED,
    RadsPerSecondSquared,
    radps2,
    -2,
    0,
    0,
    0,
    0,
    1)
NEW_UNIT_LITERAL(AngularAcceleration, RPM2, RpmSquared, rpm2, ROTATION<F> / (MINUTE<F> * MINUTE<F>))

NEW_UNIT(AngularJerk, RADIAN_PER_SECOND_CUBED, RadsPerSecondCubed, radps3, -3, 0, 0, 0, 0, 1)
NEW_UNIT_LITERAL(
    AngularJerk,
    RPM3,
    RpmCubed,
    rpm3,
    ROTATION<F> / (MINUTE<F> * MINUTE<F> * MINUTE<F>))

// Force, Pressure, Momentum, Impulse, Energy, Power
NEW_UNIT(Force, NEWTON, Newton, Ne, -2, 1, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Force, NEWTON, newton, N)

NEW_UNIT(Pressure, PASCAL, Pascal, Pa, -2, -1, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Pressure, PASCAL, pascal, Pa)

NEW_UNIT(Momentum, NEWTON_SECOND, NewtonSecond, Ns, -1, 1, 1, 0, 0, 0)
template <int F = 0>
using Impulse = Momentum<F>;

NEW_UNIT(Energy, JOULE, Joule, J, -2, 2, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Energy, JOULE, joule, J)

NEW_UNIT(Power, WATT, Watt, W, -3, 2, 1, 0, 0, 0)
UNIT_METRIC_PREFIXES_ALL(Power, WATT, watt, W)

// Radius, Curvature
NEW_UNIT(Radius, METER_PER_RADIAN, MetersPerRadian, mprad, 0, 1, 0, 0, 0, -1)

NEW_UNIT(Curvature, RADIAN_PER_METER, RadiansPerMeter, radpm, 0, -1, 0, 0, 0, 1)

using Vector2Position = modm::Vector<Length<>, 2>;
using Vector3Position = modm::Vector<Length<>, 3>;

using Vector2Velocity = modm::Vector<LinearVelocity<>, 2>;
using Vector3Velocity = modm::Vector<LinearVelocity<>, 3>;

using Vector2Acceleration = modm::Vector<LinearAcceleration<>, 2>;
using Vector3Acceleration = modm::Vector<LinearAcceleration<>, 3>;
}  // namespace tap::units
#endif  // TAPROOT_UNITS_HPP_