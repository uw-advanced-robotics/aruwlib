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

#ifndef TAPROOT_UNIT_MACROS_HPP_
#define TAPROOT_UNIT_MACROS_HPP_
#include "quantity.hpp"

/**
 * @brief Macro to define a new constant for a unit, and create conversion functions and literals
 * for it.
 */
#define NEW_UNIT_LITERAL(_name, _qname, _qfname, _qsuffix, _value)        \
    namespace constants                                                   \
    {                                                                     \
    template <int F = 0>                                                  \
    constexpr _name<F> _qname = _value;                                   \
    }                                                                     \
    namespace conversions                                                 \
    {                                                                     \
    template <int F = 0>                                                  \
    constexpr inline _name<F> from##_qfname(float value)                  \
    {                                                                     \
        return constants::_qname<F> * value;                              \
    }                                                                     \
    template <int F = 0>                                                  \
    constexpr inline float to##_qfname(_name<F> quantity)                 \
    {                                                                     \
        return quantity.valueOf() / constants::_qname<F>.valueOf();       \
    }                                                                     \
    }                                                                     \
    namespace literals                                                    \
    {                                                                     \
    constexpr _name<> operator""_##_qsuffix(long double value)            \
    {                                                                     \
        return constants::_qname<> * static_cast<float>(value);           \
    }                                                                     \
    constexpr _name<> operator""_##_qsuffix(unsigned long long int value) \
    {                                                                     \
        return constants::_qname<> * static_cast<float>(value);           \
    }                                                                     \
    }

/**
 * @brief Utility macro to create large metric prefixes (kilo, mega, giga, tera) for a unit.
 */
#define UNIT_METRIC_PREFIXES_LARGE(_name, _qname, _qfname, _qsuffix)                    \
    NEW_UNIT_LITERAL(_name, KILO##_qname, Kilo##_qfname, k##_qsuffix, _qname<F> * 1E3f) \
    NEW_UNIT_LITERAL(_name, MEGA##_qname, Mega##_qfname, M##_qsuffix, _qname<F> * 1E6f) \
    NEW_UNIT_LITERAL(_name, GIGA##_qname, Giga##_qfname, G##_qsuffix, _qname<F> * 1E9f) \
    NEW_UNIT_LITERAL(_name, TERA##_qname, Tera##_qfname, T##_qsuffix, _qname<F> * 1E12f)

/**
 * @brief Utility macro to create small metric prefixes (centi, milli, micro, nano) for a unit.
 */
#define UNIT_METRIC_PREFIXES_SMALL(_name, _qname, _qfname, _qsuffix)                       \
    NEW_UNIT_LITERAL(_name, CENTI##_qname, Centi##_qfname, c##_qsuffix, _qname<F> * 1E-2f) \
    NEW_UNIT_LITERAL(_name, MILLI##_qname, Milli##_qfname, m##_qsuffix, _qname<F> * 1E-3f) \
    NEW_UNIT_LITERAL(_name, MICRO##_qname, Micro##_qfname, u##_qsuffix, _qname<F> * 1E-6f) \
    NEW_UNIT_LITERAL(_name, NANO##_qname, Nano##_qfname, n##_qsuffix, _qname<F> * 1E-9f)

/**
 * @brief Utility macro to create metric prefixes (nano through tera) for a unit.
 */
#define UNIT_METRIC_PREFIXES_ALL(_name, _qname, _qfname, _qsuffix) \
    UNIT_METRIC_PREFIXES_LARGE(_name, _qname, _qfname, _qsuffix)   \
    UNIT_METRIC_PREFIXES_SMALL(_name, _qname, _qfname, _qsuffix)

/**
 * @brief Macro to define a new unit type. Also creates the constant with the value 1.0.
 */
#define NEW_UNIT(                             \
    _name,                                    \
    _qname,                                   \
    _qfname,                                  \
    _qsuffix,                                 \
    _time,                                    \
    _length,                                  \
    _mass,                                    \
    _current,                                 \
    _temperature,                             \
    _angle)                                   \
    template <int Frame = 0>                  \
    class _name : public Quantity<            \
                      ratio<_time>,           \
                      ratio<_length>,         \
                      ratio<_mass>,           \
                      ratio<_current>,        \
                      ratio<_temperature>,    \
                      ratio<_angle>,          \
                      Frame>                  \
    {                                         \
    public:                                   \
        explicit constexpr _name(float value) \
            : Quantity<                       \
                  ratio<_time>,               \
                  ratio<_length>,             \
                  ratio<_mass>,               \
                  ratio<_current>,            \
                  ratio<_temperature>,        \
                  ratio<_angle>,              \
                  Frame>(value)               \
        {                                     \
        }                                     \
        constexpr _name(Quantity<             \
                        ratio<_time>,         \
                        ratio<_length>,       \
                        ratio<_mass>,         \
                        ratio<_current>,      \
                        ratio<_temperature>,  \
                        ratio<_angle>,        \
                        Frame> value)         \
            : Quantity<                       \
                  ratio<_time>,               \
                  ratio<_length>,             \
                  ratio<_mass>,               \
                  ratio<_current>,            \
                  ratio<_temperature>,        \
                  ratio<_angle>,              \
                  Frame>(value)               \
        {                                     \
        }                                     \
    };                                        \
    template <int F>                          \
    struct lookupName<Quantity<               \
        ratio<_time>,                         \
        ratio<_length>,                       \
        ratio<_mass>,                         \
        ratio<_current>,                      \
        ratio<_temperature>,                  \
        ratio<_angle>,                        \
        F>>                                   \
    {                                         \
        using Named = _name<F>;               \
    };                                        \
    NEW_UNIT_LITERAL(_name, _qname, _qfname, _qsuffix, _name<F>(1.0f))
#endif