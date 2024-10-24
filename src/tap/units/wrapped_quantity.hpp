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
#include "unit_math.hpp"

namespace tap::units
{
template <isQuantity Q>
class Wrapped : public Named<Q>
{
private:
    Q lower, upper;

public:
    constexpr Wrapped(Q value, Q lower, Q upper)
        : lower(math::min(lower, upper)),
          upper(math::max(lower, upper)),
          value(wrap(value, lower, upper))
    {
    }

    template <int F = Q::Frame>
    inline Wrapped<Named<Q::Self<F>>> withSameBounds(const Q::Self<F> value) const
    {
        return Wrapped<Named<Q::Self<F>>>(
            value,
            this->lower.inOtherFrame<F>(),
            this->upper.inOtherFrame<F>());
    }

    inline bool withinRange(const Q& value) const
    {
        return (value > this->lower && value < this->upper);
    }
};
}  // namespace tap::units
#endif  // TAPROOT_WRAPPED_QUANTITY_HPP_