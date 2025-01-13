/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_FALLBACK_ENCODER_HPP_
#define TAPROOT_FALLBACK_ENCODER_HPP_

#include <array>

#include "tap/util_macros.hpp"

#include "modm/architecture/interface/assert.hpp"
#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/geometry/angle.hpp"

#include "encoder_interface.hpp"

namespace tap::motor
{
/**
 * A way to combine multiple encoders into one functional unit. The first encoder in the array
 * *must* not be null, as it is the primary encoder. All other values can be null. This encoder
 * uses the primary encoder as an initial source of truth. This allows the other encoders to align
 * themselves to it, and then have the primary encoder disconnect with the system still working.
 * This encoder is not valid until the primary encoder comes online, and stays online until either
 * all synced encoders disconnect, or the primary encoder is offline and no other encoders have
 * been synced. The positions returned by this encoder are averaged between all synced and online
 * encoders.
 */
template <uint32_t COUNT>
class FallbackEncoder : public EncoderInterface
{
public:
    FallbackEncoder(std::array<EncoderInterface*, COUNT> encoders) : encoders(encoders), seenEncoders(0)
    {
        modm_assert(this->encoders[0] != nullptr, "FallbackEncoder", "FallbackEncoder");
    }

    void initialize() override
    {
        for (EncoderInterface*& encoder : this->encoders)
        {
            if (encoder != nullptr)
            {
                encoder->initialize();
            }
        }
    }

    bool isOnline() const override
    {
        const_cast<FallbackEncoder<COUNT>*>(this)->syncEncoders();

        for (uint32_t i = 0; i < COUNT; i++)
        {
            if (this->validEncoder(i))
            {
                return true;
            }
        }

        return false;
    }

    float getPositionUnwrapped() const override
    {
        const_cast<FallbackEncoder<COUNT>*>(this)->syncEncoders();
        int onlineEncoders = 0;
        float position = 0;

        for (uint32_t i = 0; i < COUNT; i++)
        {
            if (this->validEncoder(i))
            {
                position += this->encoders[i]->getPositionUnwrapped();
                onlineEncoders += 1;
            }
        }

        return position / onlineEncoders;
    }

    float getPositionWrapped() const override
    {
        const_cast<FallbackEncoder<COUNT>*>(this)->syncEncoders();
        int onlineEncoders = 0;
        float position = 0;

        for (uint32_t i = 0; i < COUNT; i++)
        {
            if (this->validEncoder(i))
            {
                position += this->encoders[i]->getPositionWrapped();
                onlineEncoders += 1;
            }
        }

        return position / onlineEncoders;
    }

    void resetEncoderValue() override
    {
        this->syncEncoders();

        for (uint32_t i = 0; i < COUNT; i++)
        {
            if (this->validEncoder(i))
            {
                this->encoders[i]->resetEncoderValue();
            }
            else
            {
                this->seenEncoders &= ~(1 << i);
            }
        }
    }

    DISALLOW_COPY_AND_ASSIGN(FallbackEncoder)

private:
    std::array<EncoderInterface*, COUNT> encoders;
    uint32_t seenEncoders;

    void syncEncoders()
    {
        if (this->encoders[0]
                ->isOnline())  // The primary encoder *must* be online before syncing anything to it
        {
            for (uint32_t i = 1; i < COUNT; i++)
            {
                if (this->encoders[i] != nullptr && this->encoders[i]->isOnline() &&
                    !this->seenEncoder(i))
                {
                    this->seenEncoders |= 1 << i;
                    // encoders[i]->zeroTo(encoders[0]);
                }
                else if (validEncoder(i) && !seenEncoder(0))  // We reset the encoder positions but
                                                              // the primary encoder was not online
                {
                    this->seenEncoders |= 1;
                    // encoders[0]->zeroTo(encoders(i));
                }
            }
            this->seenEncoders |= 1;
        }
    }

    inline bool validEncoder(uint32_t index) const
    {
        return this->seenEncoder(index) && this->encoders[index] != nullptr &&
               this->encoders[index]->isOnline();
    }

    inline bool seenEncoder(uint32_t index) const
    {
        return ((this->seenEncoders) & (1 << index)) != 0;
    }
};

}  // namespace tap::motor

#endif  // TAPROOT_FALLBACK_ENCODER_HPP_
