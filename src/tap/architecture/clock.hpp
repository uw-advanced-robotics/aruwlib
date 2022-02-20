/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_CLOCK_HPP__
#define TAPROOT_CLOCK_HPP__
#include <cstdint>

#ifndef PLATFORM_HOSTED
#include "modm/platform.hpp"
#else
#include "modm/architecture/interface/clock.hpp"
#endif

namespace tap::arch::clock
{
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
/**
 * Object that allows you to control the global time returned by the `getTime*()` functions. Only a
 * single ClockStub may be constructed in the same scope. This is a stub designed for testing. To
 * use, declare a `ClockStub` in your test and set the `time` variable during the test when you
 * would like. The `ClockStub` you constructed will add itself as the global instance of the clock
 * stub upon construction and remove itself as the global instance when it is destructed.
 *
 * If multiple `ClockStub` instances are declared in the same scope, the program will assert and
 * crash.
 *
 * If no `ClockStub` is declared in the test's scope, the `getTime*()` functions will return 0.
 */
class ClockStub final
{
public:
    ClockStub();
    ~ClockStub();
    uint32_t time = 0;
};

uint32_t getTimeMilliseconds();
uint32_t getTimeMicroseconds();
#else
inline uint32_t getTimeMilliseconds() { return modm::Clock().now().time_since_epoch().count(); }

/**
 * @warning This clock time will wrap every 72 minutes. Do not use unless absolutely necessary.
 */
inline uint32_t getTimeMicroseconds()
{
    return modm::PreciseClock().now().time_since_epoch().count();
}
#endif
}  // namespace tap::arch::clock
#endif
