/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_MPU6500_HPP_
#define TAPROOT_MPU6500_HPP_

#include <cstdint>

#include "tap/algorithms/MahonyAHRS.h"
#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/communication/sensors/imu_heater/imu_heater.hpp"
#include "tap/util_macros.hpp"

#include "modm/math/geometry.hpp"
#include "modm/processing/protothread.hpp"

#define LITTLE_ENDIAN_INT16_TO_FLOAT(buff) \
    (static_cast<float>(static_cast<int16_t>((*(buff) << 8) | *(buff + 1))))

namespace tap
{
class Drivers;
}

namespace tap::communication::sensors::imu::mpu6500
{
/**
 * A class specifically designed for interfacing with the RoboMaster type A board Mpu6500.
 *
 * To use this class, call Remote::init() to properly initialize and calibrate
 * the MPU6500. Next, call Remote::read() to read acceleration, gyro, and temperature
 * values from the imu. Use the getter methods to access imu information.
 *
 * @note if you are shaking the imu while it is initializing, the offsets will likely
 *      be calibrated poorly and unexpectedly bad results may occur.
 */
class Mpu6500 final_mockable : public ::modm::pt::Protothread, public AbstractIMU
{
public:
    /**
     * The number of bytes read to read acceleration, gyro, and temperature.
     */
    static constexpr uint8_t ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE = 14;

    Mpu6500(Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(Mpu6500)
    mockable ~Mpu6500() = default;

    /**
     * Initialize the imu and the SPI line. Uses SPI1, which is internal to the
     * type A board.
     *
     * @note this function can block for approximately 12 seconds.
     */
    mockable void initialize(float sampleFrequency, float mahonyKp, float mahonyKi) override;
    mockable inline void init(float sampleFrequency, float mahonyKp, float mahonyKi)
    {
        initialize(sampleFrequency, mahonyKp, mahonyKi);
    }
    /**
     * Calculates the IMU's pitch, roll, and yaw angles usign the Mahony AHRS algorithm.
     * Also runs a controller to keep the temperature constant.
     * Call at 500 hz for best performance.
     */
    mockable void periodicIMUUpdate() override;

    /**
     * Read data from the imu. This is a protothread that reads the SPI bus using
     * nonblocking I/O.
     *
     * @return `true` if the function is not done, `false` otherwise
     */
    mockable bool read() override;

    /**
     * Returns the state of the IMU. Can be not connected, connected but not calibrated, calibrating
     * or calibrated. When not connected, IMU data is undefiend. When not calibrated, IMU data is
     * valid but the computed yaw angle data will drift. When calibrating, the IMU data is invalid.
     * When calibrated, the IMU data is valid and assuming proper calibration the IMU data should
     * not drift.
     *
     * To be safe, whenever you call functions that return IMU (acceleration, gyroscope,
     * temperature, and angle) data, call this function to ensure the data you are about to receive
     * is not undefined.
     */
    // mockable inline ImuState getImuState() const { return imuState; }

    virtual inline const char *getName() const { return "mpu6500"; }

    mockable inline uint32_t getPrevIMUDataReceivedTime() const { return prevIMUDataReceivedTime; }

    /**
     * Use for converting from gyro values we receive to more conventional degrees / second.
     */
    static constexpr float LSB_D_PER_S_TO_D_PER_S = 16.384f;

    inline void setCalibrationSamples(float samples) { MPU6500_OFFSET_SAMPLES = samples; }

    inline void setTargetTemperature(float temperatureC)
    {
        imuHeater.setDesiredTemperature(temperatureC);
    }

private:
    Drivers *drivers;

    static constexpr float ACCELERATION_GRAVITY = 9.80665f;

    /**
     * Use to convert the raw acceleration into more conventional degrees / second^2
     */
    static constexpr float ACCELERATION_SENSITIVITY = 4096.0f;

    inline float getAccelerationSensitivity() override { return ACCELERATION_SENSITIVITY; }


    /**
     * The number of samples we take while calibrating in order to determine the mpu offsets.
     */
    float MPU6500_OFFSET_SAMPLES = 1000;

    /**
     * The time to read the registers in nonblocking mode, in microseconds.
     */
    static constexpr int NONBLOCKING_TIME_TO_READ_REG = 450;

    /**
     * Time in ms to wait for the IMU heat to stabalize upon initialization.
     */
    static constexpr uint32_t MAX_WAIT_FOR_IMU_TEMPERATURE_STABALIZE = 10'000;

    /**
     * Time in ms to wait after IMU heat has reached stable point upon initialization.
     */
    static constexpr uint32_t WAIT_TIME_AFTER_CALIBRATION = 10'000;

    /**
     * Bit appended or removed from a register while reading/writing.
     */
    static constexpr uint8_t MPU6500_READ_BIT = 0x80;

    int delayBtwnCalcAndReadReg = 2000 - NONBLOCKING_TIME_TO_READ_REG;

    uint8_t tx = 0;  ///< Byte used for reading data in the read protothread
    uint8_t rx = 0;  ///< Byte used for reading data in the read protothread

    imu_heater::ImuHeater imuHeater;

    uint8_t txBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};
    uint8_t rxBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

    // Functions for interacting with hardware directly.

    /**
     * Pull the NSS pin low to initiate contact with the imu.
     */
    void mpuNssLow();

    /**
     * Pull the NSS pin high to end contact with the imu.
     */
    void mpuNssHigh();

    /**
     * Write to a given register.
     */
    void spiWriteRegister(uint8_t reg, uint8_t data);

    /**
     * Read from a given register.
     */
    uint8_t spiReadRegister(uint8_t reg);

    /**
     * Read from several registers.
     * regAddr is the first address read, and it reads len number of addresses
     * from that point.
     */
    void spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len);


    float parseTemp(float temperature) { return 21.0f + temperature / 333.87f; }
};

}  // namespace tap::communication::sensors::imu::mpu6500

#endif  // TAPROOT_MPU6500_HPP_
