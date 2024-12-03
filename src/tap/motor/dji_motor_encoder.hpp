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

#ifndef TAPROOT_DJI_MOTOR_ENCODER_HPP_
#define TAPROOT_DJI_MOTOR_ENCODER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/util_macros.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/geometry/angle.hpp"

#include "encoder_interface.hpp"

namespace tap::motor
{
/**
 * A class designed to interface with DJI brand motors and motor controllers over CAN.
 * This includes the C610 and C620 motor controllers and the GM6020 motor (that has a
 * built-in motor controller).
 *
 * @note: the default positive rotation direction (i.e.: when `this->isMotorInverted()
 *      == false`) is counter clockwise when looking at the shaft from the side opposite
 *      the motor. This is specified in the C620 user manual (page 18).
 *
 * DJI motor encoders store a consistent encoding for a given angle across power-cycles.
 * This means the encoder angle reported by the motor can have meaning if the encoding
 * for an angle is unique as it is for the GM6020s. However for geared motors like the
 * M3508 where a full encoder revolution does not correspond 1:1 to a shaft revolution,
 * it is impossible to know the orientation of the shaft given just the encoder value.
 *
 * Extends the CanRxListener class to attach a message handler for feedback data from the
 * motor to the CAN Rx dispatch handler.
 *
 * @note Currently there is no error handling for using a motor without having it be properly
 * initialize. You must call the `initialize` function in order for this class to work properly.
 */
class DjiMotorEncoder : public EncoderInterface
{
public:
    // 0 - 8191 for dji motors
    static constexpr uint16_t ENC_RESOLUTION = 8192;

    // Internal gear ratio of the following motors
    static constexpr float GEAR_RATIO_M3508 = 3591.0f / 187.0f;
    static constexpr float GEAR_RATIO_M3510_L1 = 3.7f / 1.0f;
    static constexpr float GEAR_RATIO_M3510_L2 = 5.2f / 1.0f;
    static constexpr float GEAR_RATIO_M3510_L3 = 19.0f / 1.0f;
    static constexpr float GEAR_RATIO_M3510_L4 = 27.0f / 1.0f;
    static constexpr float GEAR_RATIO_M2006 = 36.0f / 1.0f;

    /**
     * @param drivers a pointer to the drivers struct
     * @param desMotorIdentifier the ID of this motor controller
     * @param motorCanBus the CAN bus the motor is on
     * @param isInverted if `false` the positive rotation direction of the shaft is
     *      counter-clockwise when looking at the shaft from the side opposite the motor.
     *      If `true` then the positive rotation direction will be clockwise.
     * @param name a name to associate with the motor for use in the motor menu
     * @param encoderWrapped the starting encoderValue to store for this motor.
     *      Will be overwritten by the first reported encoder value from the motor
     * @param encoderRevolutions the starting number of encoder revolutions to store.
     *      See comment for DjiMotor::encoderRevolutions for more details.
     */
    DjiMotorEncoder(
        bool isInverted,
        uint16_t encoderWrapped = ENC_RESOLUTION / 2,
        int64_t encoderRevolutions = 0);

    void initialize() override {};

    bool isOnline() const override;

    float getPositionUnwrapped() const override;

    float getPositionWrapped() const override;

    int64_t getEncoderUnwrapped() const;

    uint16_t getEncoderWrapped() const;

    /**
     * Resets this motor's current encoder home position to the current encoder position reported by
     * CAN messages, and resets this motor's encoder revolutions to 0.
     */
    void resetEncoderValue() override;

    DISALLOW_COPY_AND_ASSIGN(DjiMotorEncoder)

    /**
     * Overrides virtual method in the can class, called every time a message with the
     * CAN message id this class is attached to is received by the can receive handler.
     * Parses the data in the message and updates this class's fields accordingly.
     *
     * @param[in] message the message to be processed.
     */
    void processMessage(const modm::can::Message& message);

    template <typename T>
    static void assertEncoderType()
    {
        constexpr bool good_type =
            std::is_same<typename std::decay<T>::type, std::int64_t>::value ||
            std::is_same<typename std::decay<T>::type, std::uint16_t>::value;
        static_assert(good_type, "x is not of the correct type");
    }

    template <typename T>
    static T degreesToEncoder(float angle)
    {
        assertEncoderType<T>();
        return static_cast<T>((ENC_RESOLUTION * angle) / 360);
    }

    template <typename T>
    static float encoderToDegrees(T encoder)
    {
        assertEncoderType<T>();
        return (360.0f * static_cast<float>(encoder)) / ENC_RESOLUTION;
    }

private:
    /**
     * Updates the stored encoder value given a newly received encoder value
     * special logic necessary for keeping track of unwrapped encoder value.
     */
    void updateEncoderValue(uint16_t newEncWrapped);

    /**
     * If `false` the positive rotation direction of the shaft is counter-clockwise when
     * looking at the shaft from the side opposite the motor. If `true` then the positive
     * rotation direction will be clockwise.
     */
    bool motorInverted;

    /**
     * The raw encoder value reported by the motor controller relative to
     * encoderHomePosition. It wraps around from {0..8191}, hence "Wrapped"
     */
    uint16_t encoderWrapped;

    /**
     * Absolute unwrapped encoder position =
     *      encoderRevolutions * ENCODER_RESOLUTION + encoderWrapped
     * This lets us keep track of some sense of absolute position even while
     * raw encoderValue continuosly loops within {0..8191}. Origin value is
     * arbitrary.
     */
    int64_t encoderRevolutions;

    /**
     * The actual encoder wrapped value received from CAN messages where this motor
     * is considered to have an encoder value of 0. encoderHomePosition is 0 by default.
     */
    uint16_t encoderHomePosition;

    // wait time before the motor is considered disconnected, in milliseconds
    static const uint32_t MOTOR_DISCONNECT_TIME = 100;

    tap::arch::MilliTimeout encoderDisconnectTimeout;
};

}  // namespace tap::motor

#endif  // TAPROOT_DJI_MOTOR_ENCODER_HPP_
