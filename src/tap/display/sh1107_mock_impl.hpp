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

#ifndef TAPROOT_SH1107_HPP_
#error "Don't include this file directly, use 'sh1107.hpp' instead!"
#endif

template <unsigned int Width, unsigned int Height, bool Flipped, unsigned int RotateDegrees>
modm::ResumableResult<bool> tap::display::Sh1107<Width, Height, Flipped, RotateDegrees>::updateNonblocking()
{
    RF_BEGIN(0);
    // no-op
    RF_END_RETURN(false);
}

template <unsigned int Width, unsigned int Height, bool Flipped, unsigned int RotateDegrees>
void tap::display::Sh1107<Width, Height, Flipped, RotateDegrees>::update()
{
    // no-op
}

template <unsigned int Width, unsigned int Height, bool Flipped, unsigned int RotateDegrees>
void tap::display::Sh1107<Width, Height, Flipped, RotateDegrees>::setInvert(bool invert)
{
    // no-op
}

// ----------------------------------------------------------------------------
template <unsigned int Width, unsigned int Height, bool Flipped, unsigned int RotateDegrees>
void tap::display::Sh1107<Width, Height, Flipped, RotateDegrees>::initializeBlocking()
{
    this->clear();
    this->update();
}
