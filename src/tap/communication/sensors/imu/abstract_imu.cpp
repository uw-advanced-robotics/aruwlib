/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "abstract_imu.hpp"

namespace tap::communication::sensors::imu {

void AbstractIMU::initialize(float sampleFrequency, float mahonyKp, float mahonyKi) {
    mahonyAlgorithm.begin(sampleFrequency, mahonyKp, mahonyKi);
    imuState = ImuState::IMU_NOT_CALIBRATED;
}

void AbstractIMU::requestCalibration() {
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED) {
        resetOffsets();
        calibrationSample = 0;
        imuState = ImuState::IMU_CALIBRATING;
    }
}


void AbstractIMU::setMountingTransform(const Transform& transform) {
    mountingTransform = transform;
}


void AbstractIMU::periodicIMUUpdate() {
    if (imuState == ImuState::IMU_CALIBRATED) {
        // Update Mahony algorithm
        mahonyAlgorithm.updateIMU(
            imuData.gyroDegPerSec[ImuData::X],
            imuData.gyroDegPerSec[ImuData::Y],
            imuData.gyroDegPerSec[ImuData::Z],
            imuData.accG[ImuData::X],
            imuData.accG[ImuData::Y],
            imuData.accG[ImuData::Z]);

    } else if (imuState == ImuState::IMU_CALIBRATING) {
        computeOffsets();
    }
}

void AbstractIMU::resetOffsets(){
    for(int i = 0; i < 3; i++){
        imuData.accOffsetRaw[i] = 0;
        imuData.gyroOffsetRaw[i] = 0;
    }
}

void AbstractIMU::computeOffsets(){
    calibrationSample++;

    imuData.gyroOffsetRaw[ImuData::X] += imuData.gyroRaw[ImuData::X];
    imuData.gyroOffsetRaw[ImuData::Y] += imuData.gyroRaw[ImuData::Y];
    imuData.gyroOffsetRaw[ImuData::Z] += imuData.gyroRaw[ImuData::Z];
    imuData.accOffsetRaw[ImuData::X] += imuData.accRaw[ImuData::X];
    imuData.accOffsetRaw[ImuData::Y] += imuData.accRaw[ImuData::Y];
    imuData.accOffsetRaw[ImuData::Z] += imuData.accRaw[ImuData::Z] - getAccelerationSensitivity();

    if (calibrationSample >= offsetSampleCount)
    {
        calibrationSample = 0;
        imuData.gyroOffsetRaw[ImuData::X] /= offsetSampleCount;
        imuData.gyroOffsetRaw[ImuData::Y] /= offsetSampleCount;
        imuData.gyroOffsetRaw[ImuData::Z] /= offsetSampleCount;
        imuData.accOffsetRaw[ImuData::X] /= offsetSampleCount;
        imuData.accOffsetRaw[ImuData::Y] /= offsetSampleCount;
        imuData.accOffsetRaw[ImuData::Z] /= offsetSampleCount;
        imuState = ImuState::IMU_CALIBRATED;
        mahonyAlgorithm.reset();
    }
}

}  // namespace tap::communication::sensors::imu
