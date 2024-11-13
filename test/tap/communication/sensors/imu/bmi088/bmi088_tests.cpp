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

#include <gtest/gtest.h>

#include "tap/communication/sensors/imu/bmi088/bmi088.hpp"
#include "tap/communication/sensors/imu/bmi088/bmi088_hal.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::sensors::imu::bmi088;

TEST(Bmi088, periodicIMUUpdate_initialize_not_called_errors)
{
    tap::Drivers drivers;
    Bmi088 bmi088(&drivers);

    EXPECT_CALL(drivers.errorController, addToErrorList);

    bmi088.periodicIMUUpdate();
}

static void initializeBmi088(Bmi088 &bmi088)
{
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccWriteSingleReg();
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccReadSingleReg(Bmi088Data::Acc::ACC_CHIP_ID_VALUE);
    Bmi088Hal::expectAccWriteSingleReg();
    Bmi088Hal::expectAccReadSingleReg(171);  // acc config
    Bmi088Hal::expectAccWriteSingleReg();
    Bmi088Hal::expectAccReadSingleReg(0);  // acc range

    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(Bmi088Data::Gyro::GYRO_CHIP_ID_VALUE);
    Bmi088Hal::expectGyroReadSingleReg(Bmi088Data::Gyro::GYRO_CHIP_ID_VALUE);
    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(0);  // gyro range
    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(130);  // gyro bandwidth
    Bmi088Hal::expectGyroWriteSingleReg();
    Bmi088Hal::expectGyroReadSingleReg(0);  // gyro powermode

    bmi088.initialize(1000, 0, 0);

    Bmi088Hal::clearData();
}

TEST(Bmi088, periodicIMUUpdate_initialize_called_no_errors)
{
    tap::Drivers drivers;
    Bmi088 bmi088(&drivers);

    initializeBmi088(bmi088);

    bmi088.read();
    bmi088.periodicIMUUpdate();

    EXPECT_EQ(Bmi088::ImuState::IMU_NOT_CALIBRATED, bmi088.getImuState());
}

TEST(Bmi088, periodicIMUUpdate_gyro_acc_temp_data_parsed_properly)
{
    tap::Drivers drivers;
    Bmi088 bmi088(&drivers);

    initializeBmi088(bmi088);

    struct
    {
        int16_t x = 0x1234;
        int16_t y = 0x4321;
        int16_t z = 0x3214;
    } modm_packed accData;

    struct
    {
        int16_t x = 0x6789;
        int16_t y = 0x9876;
        int16_t z = 0x8769;
    } modm_packed gyroData;

    Bmi088Hal::expectAccMultiRead(reinterpret_cast<uint8_t *>(&accData), sizeof(accData));
    Bmi088Hal::expectGyroMultiRead(reinterpret_cast<uint8_t *>(&gyroData), sizeof(gyroData));

    bmi088.read();
    bmi088.periodicIMUUpdate();

    static constexpr float ALPHA = 1E-3;
    EXPECT_NEAR(accData.x * Bmi088::ACC_G_PER_ACC_COUNT, bmi088.getAx(), ALPHA);
    EXPECT_NEAR(accData.y * Bmi088::ACC_G_PER_ACC_COUNT, bmi088.getAy(), ALPHA);
    EXPECT_NEAR(accData.z * Bmi088::ACC_G_PER_ACC_COUNT, bmi088.getAz(), ALPHA);
    EXPECT_NEAR(gyroData.x * Bmi088::GYRO_DS_PER_GYRO_COUNT, bmi088.getGx(), ALPHA);
    EXPECT_NEAR(gyroData.y * Bmi088::GYRO_DS_PER_GYRO_COUNT, bmi088.getGy(), ALPHA);
    EXPECT_NEAR(gyroData.z * Bmi088::GYRO_DS_PER_GYRO_COUNT, bmi088.getGz(), ALPHA);
}

TEST(Bmi088, requestRecalibration__does_nothing_when_imu_disconnected)
{
    tap::Drivers drivers;
    Bmi088 bmi088(&drivers);

    bmi088.requestRecalibration();

    EXPECT_EQ(Bmi088::ImuState::IMU_NOT_CONNECTED, bmi088.getImuState());
}

TEST(Bmi088, requestRecalibration__calibration_adds_offset_to_acc_gyro_data)
{
    tap::Drivers drivers;
    Bmi088 bmi088(&drivers);

    initializeBmi088(bmi088);

    bmi088.requestRecalibration();

    struct
    {
        int16_t x = 12;
        int16_t y = 15;
        int16_t z = 20 + tap::algorithms::ACCELERATION_GRAVITY.valueOf() / Bmi088::ACC_G_PER_ACC_COUNT;
    } modm_packed accData;

    struct
    {
        int16_t x = -14;
        int16_t y = 3;
        int16_t z = 8;
    } modm_packed gyroData;

    for (int i = 0; i < 3000; i++)
    {
        Bmi088Hal::expectAccMultiRead(reinterpret_cast<uint8_t *>(&accData), sizeof(accData));
        Bmi088Hal::expectGyroMultiRead(reinterpret_cast<uint8_t *>(&gyroData), sizeof(gyroData));
        bmi088.read();
        bmi088.periodicIMUUpdate();
    }

    accData.x = 0;
    accData.y = 0;
    accData.z = tap::algorithms::ACCELERATION_GRAVITY.valueOf() / Bmi088::ACC_G_PER_ACC_COUNT;
    gyroData.x = 0;
    gyroData.y = 0;
    gyroData.z = 0;

    Bmi088Hal::expectAccMultiRead(reinterpret_cast<uint8_t *>(&accData), sizeof(accData));
    Bmi088Hal::expectGyroMultiRead(reinterpret_cast<uint8_t *>(&gyroData), sizeof(gyroData));
    bmi088.read();
    bmi088.periodicIMUUpdate();

    static constexpr float ALPHA = 1E-3;
    EXPECT_NEAR(-12 * Bmi088::ACC_G_PER_ACC_COUNT, bmi088.getAx(), ALPHA);
    EXPECT_NEAR(-15 * Bmi088::ACC_G_PER_ACC_COUNT, bmi088.getAy(), ALPHA);
    EXPECT_NEAR(
        (-20 + tap::algorithms::ACCELERATION_GRAVITY.valueOf() / Bmi088::ACC_G_PER_ACC_COUNT) *
            Bmi088::ACC_G_PER_ACC_COUNT,
        bmi088.getAz(),
        ALPHA);
    EXPECT_NEAR(14 * Bmi088::GYRO_DS_PER_GYRO_COUNT, bmi088.getGx(), ALPHA);
    EXPECT_NEAR(-3 * Bmi088::GYRO_DS_PER_GYRO_COUNT, bmi088.getGy(), ALPHA);
    EXPECT_NEAR(-8 * Bmi088::GYRO_DS_PER_GYRO_COUNT, bmi088.getGz(), ALPHA);

    EXPECT_EQ(Bmi088::ImuState::IMU_CALIBRATED, bmi088.getImuState());
}
