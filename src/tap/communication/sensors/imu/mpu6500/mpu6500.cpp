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
      processRawMpu6500DataFn(Mpu6500::defaultProcessRawMpu6500Data),
      imuHeater(drivers)
{
}

void Mpu6500::requestCalibration()
{
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        imuData.gyroOffsetRaw[ImuData::X] = 0;
        imuData.gyroOffsetRaw[ImuData::Y] = 0;
        imuData.gyroOffsetRaw[ImuData::Z] = 0;
        imuData.accOffsetRaw[ImuData::X] = 0;
        imuData.accOffsetRaw[ImuData::Y] = 0;
        imuData.accOffsetRaw[ImuData::Z] = 0;
        calibrationSample = 0;
        imuState = ImuState::IMU_CALIBRATING;
    }
}

void Mpu6500::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    mahonyAlgorithm.begin(sampleFrequency, mahonyKp, mahonyKi);
    imuHeater.initialize();
    imuState = ImuState::IMU_NOT_CALIBRATED;
    init(sampleFrequency, mahonyKp, mahonyKi);
}

void Mpu6500::init(float sampleFrequency, float mahonyKp, float mahonyKi)
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

    readRegistersTimeout.restart(delayBtwnCalcAndReadReg);

    mahonyAlgorithm.begin(sampleFrequency, mahonyKp, mahonyKi);

    imuState = ImuState::IMU_NOT_CALIBRATED;
}

void Mpu6500::resetOffsets()
{
    imuData.gyroOffsetRaw[ImuData::X] = 0;
    imuData.gyroOffsetRaw[ImuData::Y] = 0;
    imuData.gyroOffsetRaw[ImuData::Z] = 0;
    imuData.accOffsetRaw[ImuData::X] = 0;
    imuData.accOffsetRaw[ImuData::Y] = 0;
    imuData.accOffsetRaw[ImuData::Z] = 0;
}

void Mpu6500::computeOffsets()
{
    calibrationSample++;

    imuData.gyroOffsetRaw[ImuData::X] += imuData.gyroRaw[ImuData::X];
    imuData.gyroOffsetRaw[ImuData::Y] += imuData.gyroRaw[ImuData::Y];
    imuData.gyroOffsetRaw[ImuData::Z] += imuData.gyroRaw[ImuData::Z];
    imuData.accOffsetRaw[ImuData::X] += imuData.accRaw[ImuData::X];
    imuData.accOffsetRaw[ImuData::Y] += imuData.accRaw[ImuData::Y];
    imuData.accOffsetRaw[ImuData::Z] += imuData.accRaw[ImuData::Z] - ACCELERATION_SENSITIVITY;

    if (calibrationSample >= MPU6500_OFFSET_SAMPLES)
    {
        calibrationSample = 0;
        imuData.gyroOffsetRaw[ImuData::X] /= MPU6500_OFFSET_SAMPLES;
        imuData.gyroOffsetRaw[ImuData::Y] /= MPU6500_OFFSET_SAMPLES;
        imuData.gyroOffsetRaw[ImuData::Z] /= MPU6500_OFFSET_SAMPLES;
        imuData.accOffsetRaw[ImuData::X] /= MPU6500_OFFSET_SAMPLES;
        imuData.accOffsetRaw[ImuData::Y] /= MPU6500_OFFSET_SAMPLES;
        imuData.accOffsetRaw[ImuData::Z] /= MPU6500_OFFSET_SAMPLES;
        imuState = ImuState::IMU_CALIBRATED;
        mahonyAlgorithm.reset();
    }
}

void Mpu6500::periodicIMUUpdate()
{
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
        tiltAngleCalculated = false;
        // Start reading registers in DELAY_BTWN_CALC_AND_READ_REG us
    }
    else
    {
        calibrationSample++;

        imuData.gyroOffsetRaw[ImuData::X] += imuData.gyroRaw[ImuData::X];
        imuData.gyroOffsetRaw[ImuData::Y] += imuData.gyroRaw[ImuData::Y];
        imuData.gyroOffsetRaw[ImuData::Z] += imuData.gyroRaw[ImuData::Z];
        imuData.accOffsetRaw[ImuData::X] += imuData.accRaw[ImuData::X];
        imuData.accOffsetRaw[ImuData::Y] += imuData.accRaw[ImuData::Y];
        imuData.accOffsetRaw[ImuData::Z] += imuData.accRaw[ImuData::Z] - ACCELERATION_SENSITIVITY;

        if (calibrationSample >= MPU6500_OFFSET_SAMPLES)
        {
            calibrationSample = 0;
            imuData.gyroOffsetRaw[ImuData::X] /= MPU6500_OFFSET_SAMPLES;
            imuData.gyroOffsetRaw[ImuData::Y] /= MPU6500_OFFSET_SAMPLES;
            imuData.gyroOffsetRaw[ImuData::Z] /= MPU6500_OFFSET_SAMPLES;
            imuData.accOffsetRaw[ImuData::X] /= MPU6500_OFFSET_SAMPLES;
            imuData.accOffsetRaw[ImuData::Y] /= MPU6500_OFFSET_SAMPLES;
            imuData.accOffsetRaw[ImuData::Z] /= MPU6500_OFFSET_SAMPLES;
            imuState = ImuState::IMU_CALIBRATED;
            mahonyAlgorithm.reset();
        }
    }

    readRegistersTimeout.restart(delayBtwnCalcAndReadReg);

    imuHeater.runTemperatureController(getTemp());

    addValidationErrors();
}

bool Mpu6500::read()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();
    while (true)
    {
        PT_WAIT_UNTIL(readRegistersTimeout.execute());

        mpuNssLow();
        tx = MPU6500_ACCEL_XOUT_H | MPU6500_READ_BIT;
        rx = 0;
        txBuff[0] = tx;
        PT_CALL(Board::ImuSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::ImuSpiMaster::transfer(txBuff, rxBuff, ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE));
        mpuNssHigh();

        (*processRawMpu6500DataFn)(rxBuff, imuData.accRaw, imuData.gyroRaw);

        imuData.temperature = rxBuff[6] << 8 | rxBuff[7];

        prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();
    }
    PT_END();
#else
    return false;
#endif
}

float Mpu6500::getTiltAngle()
{
    if (!tiltAngleCalculated)
    {
        tiltAngle = modm::toDegree(acosf(
            cosf(mahonyAlgorithm.getPitchRadians()) * cosf(mahonyAlgorithm.getRollRadians())));
        tiltAngleCalculated = true;
    }
    return validateReading(tiltAngle);
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

/**
 * Add any errors to the error handler that have came up due to calls to validateReading.
 */
void Mpu6500::addValidationErrors()
{
    if (errorState & (1 << static_cast<uint8_t>(ImuState::IMU_NOT_CALIBRATED)))
    {
        RAISE_ERROR(drivers, "IMU data requested but IMU not calibrated");
    }
    else if (errorState & (1 << static_cast<uint8_t>(ImuState::IMU_CALIBRATING)))
    {
        RAISE_ERROR(drivers, "Reading IMU data but IMU calibrating");
    }
    else if (errorState & (1 << static_cast<uint8_t>(ImuState::IMU_NOT_CONNECTED)))
    {
        RAISE_ERROR(drivers, "Failed to initialize IMU properly");
    }

    errorState = 0;
}

void Mpu6500::defaultProcessRawMpu6500Data(
    const uint8_t (&rxBuff)[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE],
    modm::Vector3f &accel,
    modm::Vector3f &gyro)
{
    accel.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    accel.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    accel.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    gyro.x = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 8);
    gyro.y = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 10);
    gyro.z = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 12);
}

}  // namespace tap::communication::sensors::imu::mpu6500
