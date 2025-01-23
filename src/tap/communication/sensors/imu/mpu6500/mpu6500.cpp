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

#include "mpu6500.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/board/board.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "mpu6500_config.hpp"
#include "mpu6500_reg.hpp"

using namespace modm::literals;
using namespace tap::arch;

namespace tap::communication::sensors::imu::mpu6500
{
Mpu6500::Mpu6500(Drivers *drivers)
    : AbstractIMU(drivers),
      drivers(drivers),
      imuHeater(drivers)
{
}

void Mpu6500::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
#ifndef PLATFORM_HOSTED
    // Configure NSS pin
    Board::ImuNss::GpioOutput();

    // connect GPIO pins to the alternate SPI function
    Board::ImuSpiMaster::connect<Board::ImuMiso::Miso, Board::ImuMosi::Mosi, Board::ImuSck::Sck>();

    // initialize SPI with clock speed
    Board::ImuSpiMaster::initialize<Board::SystemClock, 703125_Hz>();

    // See page 42 of the mpu6500 register map for initialization process:
    // https://3cfeqx1hf82y3xcoull08ihx-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf
    //
    // When using SPI interface, user should use PWR_MGMT_1 (register 107) as well as
    // SIGNAL_PATH_RESET (register 104) to ensure the reset is performed properly. The sequence
    // used should be:
    //  1. Set H_RESET = 1 (register PWR_MGMT_1)
    //  2. Wait 100ms
    //  3. Set GYRO_RST = ACCEL_RST = TEMP_RST = 1 (register SIGNAL_PATH_RESET)
    //  4. Wait 100ms

    // set power mode
    spiWriteRegister(MPU6500_PWR_MGMT_1, MPU6500_PWR_MGMT_1_DEVICE_RESET_BIT);

    modm::delay_ms(100);

    // reset gyro, accel, and temperature
    spiWriteRegister(MPU6500_SIGNAL_PATH_RESET, MPU6500_SIGNAL_PATH_RESET_ALL);

    modm::delay_ms(100);

    // verify mpu register ID
    if (MPU6500_ID != spiReadRegister(MPU6500_WHO_AM_I))
    {
        RAISE_ERROR(drivers, "Failed to initialize the IMU properly");
        return;
    }

    // Configure mpu
    spiWriteRegister(MPU6500_PWR_MGMT_1, MPU6500_PWR_MGMT_1_CLKSEL);
    modm::delay_ms(1);  // Delay for some time to wait for the register to be updated (probably not
                        // necessary but we do it anyway)
    spiWriteRegister(MPU6500_PWR_MGMT_2, 0x00);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_CONFIG, MPU6500_CONFIG_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_GYRO_CONFIG, MPU6500_GYRO_CONFIG_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_CONFIG_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_ACCEL_CONFIG_2, MPU6500_ACCEL_CONFIG_2_DATA);
    modm::delay_ms(1);
    spiWriteRegister(MPU6500_USER_CTRL, MPU6500_USER_CTRL_DATA);
    modm::delay_ms(1);
#endif

    imuHeater.initialize();

    delayBtwnCalcAndReadReg =
        static_cast<int>(1e6f / sampleFrequency) - NONBLOCKING_TIME_TO_READ_REG;

    assert(delayBtwnCalcAndReadReg >= 0);

    readTimeout.restart(delayBtwnCalcAndReadReg);

    AbstractIMU::initialize(sampleFrequency, mahonyKp, mahonyKi);
}

void Mpu6500::periodicIMUUpdate()
{
    AbstractIMU::periodicIMUUpdate();

    readTimeout.restart(delayBtwnCalcAndReadReg);

    imuHeater.runTemperatureController(getTemp());
}

void Mpu6500::processRawData(const uint8_t (&rxBuff)[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE]) {
    modm::Vector3f rawAccel(
        LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff),
        LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2),
        LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4)
    );

    modm::Vector3f rawGyro(
        LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 8),
        LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 10),
        LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 12)
    );

    auto transformedAccel = mountingTransform.apply(
        Orientation(
            rawAccel.x,
            rawAccel.y,
            rawAccel.z
        )
    );
    auto transformedGyro = mountingTransform.apply(
        Orientation(
            rawGyro.x,
            rawGyro.y,
            rawGyro.z
        )
    );

    auto transformedGravity = mountingTransform.apply(
        Orientation(
            0.0f,
            0.0f,
            ACCELERATION_GRAVITY
        )
    );

    transformedAccel -= transformedGravity;

    imuData.accG[ImuData::X] = transformedAccel.x;
    imuData.accG[ImuData::Y] = transformedAccel.y;
    imuData.accG[ImuData::Z] = transformedAccel.z;

    imuData.gyroDegPerSec[ImuData::X] = transformedGyro.x;
    imuData.gyroDegPerSec[ImuData::Y] = transformedGyro.y;
    imuData.gyroDegPerSec[ImuData::Z] = transformedGyro.z;
}

bool Mpu6500::read()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();
    while (true)
    {
        PT_WAIT_UNTIL(readTimeout.execute());

        mpuNssLow();
        tx = MPU6500_ACCEL_XOUT_H | MPU6500_READ_BIT;
        rx = 0;
        txBuff[0] = tx;
        PT_CALL(Board::ImuSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::ImuSpiMaster::transfer(txBuff, rxBuff, ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE));
        mpuNssHigh();

        imuData.accRaw[0] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
        imuData.accRaw[1] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
        imuData.accRaw[2] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

        imuData.accRaw[0] = (imuData.accRaw[0] - imuData.accOffsetRaw[ImuData::X]) * 
                            ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY;
        imuData.accRaw[1] = (imuData.accRaw[1] - imuData.accOffsetRaw[ImuData::Y]) * 
                            ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY;
        imuData.accRaw[2] = (imuData.accRaw[2] - imuData.accOffsetRaw[ImuData::Z]) * 
                            ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY;

        imuData.gyroRaw[0] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 8);
        imuData.gyroRaw[1] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 10);
        imuData.gyroRaw[2] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 12);

        imuData.gyroRaw[0] = (imuData.gyroRaw[0] - imuData.gyroOffsetRaw[ImuData::X]) / 
                             LSB_D_PER_S_TO_D_PER_S;
        imuData.gyroRaw[1] = (imuData.gyroRaw[1] - imuData.gyroOffsetRaw[ImuData::Y]) / 
                             LSB_D_PER_S_TO_D_PER_S;
        imuData.gyroRaw[2] = (imuData.gyroRaw[2] - imuData.gyroOffsetRaw[ImuData::Z]) / 
                             LSB_D_PER_S_TO_D_PER_S;

        imuData.temperature = parseTemp(static_cast<float>(rxBuff[6] << 8 | rxBuff[7]));

        prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();
    }
    PT_END();
#else
    return false;
#endif
}



// Hardware interface functions (blocking functions, for initialization only)

void Mpu6500::spiWriteRegister(uint8_t reg, uint8_t data)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    UNUSED(data);
#else
    mpuNssLow();
    uint8_t tx = reg & ~MPU6500_READ_BIT;
    uint8_t rx = 0;  // Unused
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    tx = data;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
#endif
}

uint8_t Mpu6500::spiReadRegister(uint8_t reg)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    return 0;
#else
    mpuNssLow();
    uint8_t tx = reg | MPU6500_READ_BIT;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
    return rx;
#endif
}

void Mpu6500::spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len)
{
#ifdef PLATFORM_HOSTED
    UNUSED(regAddr);
    UNUSED(pData);
    UNUSED(len);
#else
    mpuNssLow();
    uint8_t tx = regAddr | MPU6500_READ_BIT;
    uint8_t rx = 0;
    txBuff[0] = tx;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(txBuff, pData, len);
    mpuNssHigh();
#endif
}

void Mpu6500::mpuNssLow()
{
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::Low);
#endif
}

void Mpu6500::mpuNssHigh()
{
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::High);
#endif
}


}  // namespace tap::communication::sensors::imu::mpu6500
