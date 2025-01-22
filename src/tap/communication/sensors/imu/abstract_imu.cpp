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

void AbstractIMU::periodicIMUUpdate() {
    if (imuState == ImuState::IMU_CALIBRATED) {
        mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
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

    imuData.gyroOffsetRaw.x += imuData.gyro[ImuData::X];
    imuData.gyroOffsetRaw.y += imuData.gyro[ImuData::Y];
    imuData.gyroOffsetRaw.z += imuData.gyro[ImuData::Z];
    imuData.accOffsetRaw.x += imuData.accRaw[ImuData::X];
    imuData.accOffsetRaw.y += imuData.accRaw[ImuData::Y];
    imuData.accOffsetRaw.z += imuData.accRaw.z - getAccelerationSenstivity();

    if (calibrationSample >= MPU6500_OFFSET_SAMPLES)
    {
        calibrationSample = 0;
        imuData.gyroOffsetRaw.x /= MPU6500_OFFSET_SAMPLES;
        imuData.gyroOffsetRaw.y /= MPU6500_OFFSET_SAMPLES;
        imuData.gyroOffsetRaw.z /= MPU6500_OFFSET_SAMPLES;
        imuData.accelOffset.x /= MPU6500_OFFSET_SAMPLES;
        imuData.accelOffset.y /= MPU6500_OFFSET_SAMPLES;
        imuData.accelOffset.z /= MPU6500_OFFSET_SAMPLES;
        imuState = ImuState::IMU_CALIBRATED;
        mahonyAlgorithm.reset();
    }
}

}  // namespace tap::communication::sensors::imu
