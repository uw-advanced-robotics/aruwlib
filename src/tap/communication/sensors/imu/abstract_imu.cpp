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
    if (imuState == ImuState::IMU_NOT_CONNECTED)
    {
        RAISE_ERROR(drivers, "periodicIMUUpdate called w/ imu not connected");
        return;
    }
    if (imuState == ImuState::IMU_CALIBRATED) {
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

    imuData.gyroOffsetRaw.x += imuData.gyroRaw[ImuData::X];
    imuData.gyroOffsetRaw.y += imuData.gyroRaw[ImuData::Y];
    imuData.gyroOffsetRaw.z += imuData.gyroRaw[ImuData::Z];
    imuData.accOffsetRaw.x += imuData.accRaw[ImuData::X];
    imuData.accOffsetRaw.y += imuData.accRaw[ImuData::Y];
    imuData.accOffsetRaw.z += imuData.accRaw.z - getAccelerationSenstivity();

    if (calibrationSample >= offsetSampleCount)
    {
        calibrationSample = 0;
        imuData.gyroOffsetRaw.x /= offsetSampleCount;
        imuData.gyroOffsetRaw.y /= offsetSampleCount;
        imuData.gyroOffsetRaw.z /= offsetSampleCount;
        imuData.accelOffset.x /= offsetSampleCount;
        imuData.accelOffset.y /= offsetSampleCount;
        imuData.accelOffset.z /= offsetSampleCount;
        imuState = ImuState::IMU_CALIBRATED;
        mahonyAlgorithm.reset();
    }
}

}  // namespace tap::communication::sensors::imu
