/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CRC_HPP_
#define CRC_HPP_

#include <cstdint>

namespace aruwlib
{
namespace algorithms
{
#define CRC8_INIT 0xff
#define CRC16_INIT 0xffff

/**
 * Fast crc8 calculation using a lookup table. The crc looks at messageLength
 * bytes in the message to calculate the crc.
 *
 * @param[in] message the message to be used for calculation.
 * @param[in] messageLength the number of bytes to look at when calculating the crc.
 * @param[in] initCRC8 normally leave as CRC8_INIT.
 * @return the calculated crc.
 */
uint8_t calculateCRC8(const uint8_t *message, uint32_t messageLength, uint8_t initCRC8 = CRC8_INIT);

/**
 * Fast crc16 calculation using a lookup table.
 *
 * @see calculateCRC8
 *
 * @param[in] message the message to be used for calculation.
 * @param[in] messageLength the number of bytes to look at when calculating the crc.
 * @param[in] initCRC8 normally leave as CRC8_INIT.
 * @return the calculated crc.
 */
uint16_t calculateCRC16(
    const uint8_t *message,
    uint32_t messageLength,
    uint16_t initCRC16 = CRC16_INIT);

}  // namespace algorithms

}  // namespace aruwlib

#endif  // CRC_HPP_
