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

#ifndef TAPROOT_MOTOR_SIM_HPP_
#define TAPROOT_MOTOR_SIM_HPP_

#ifdef PLATFORM_HOSTED

#include <cmath>
#include <cstdint>

namespace tap::motor::motorsim
{
class MotorSim
{
public:
    struct Config
    {
        int16_t maxInputMag;  ///< Integer
        int16_t maxencoder;   ///< Integer
        float maxCurrent;     ///< Amps
        float currentLim;     ///< Amps
        float maxW;           ///< RPM
        float kt;             ///< (N*m)/A
        float wtGrad;         ///< RPM/(N*m)
    };

    static constexpr Config M3508_CONFIG = {
        .maxInputMag = 16'384,
        .maxencoder = 8'192,
        .maxCurrent = 20,
        .currentLim = 10,
        .maxW = 469,
        .kt = 0.3f,
        .wtGrad = 72,
    };

    MotorSim(const Config &config);

    /**
     * Resets the dynamic values in the motor simulator. Should be run before any simulation.
     */
    void reset();

    /**
     * Sets the motor input (as an integer) used for simulation.
     * Should be updated every cycle. Default input is 0.
     * Function will do nothing if input is invalid.
     */
    void setMotorInput(int16_t in);

    /**
     * Sets the load on the motor (in N*m) used for simulation. The default load value is 0 N*m.
     */
    void setLoad(float load);

    /**
     * Updates the relevant quantities for the motor being simulated.
     * Must be run iteratively in order for getEnc() and getRPM() to work correctly.
     */
    void update();

    /**
     * Returns the current (in amps) given to the GM3508 for the given input.
     */
    float getCurrent() const;

    /**
     * Returns the angular position of the motor.
     */
    int16_t getEnc() const;

    /**
     * Returns the input integer given to the motor by the CAN messages.
     */
    int16_t getInput() const;

    /**
     * Returns the maximum torque (in N*m) of the motor
     */
    float getMaxTorque() const;

    /**
     * Returns the current rotational speed of the motor in RPM.
     */
    int16_t getRPM() const;

private:
    const Config config;

    /* Class Variables */
    float load = 0;  // N*m
    int16_t enc = 0;
    float rpm = 0;
    int16_t input = 0;
    uint32_t prevTime = 0;
};
}  // namespace tap::motor::motorsim

#endif  // PLATFORM_HOSTED

#endif  // TAPROOT_MOTOR_SIM_HPP_
