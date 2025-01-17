#include "abstract_imu.hpp"

namespace tap::communication::sensors::imu {

void AbstractIMU::initialize(float sampleFrequency, float mahonyKp, float mahonyKi) {
    mahonyAlgorithm.begin(sampleFrequency, mahonyKp, mahonyKi);
    imuHeater.initialize();
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
        imuHeater.runTemperatureController(getTemp());
    } else if (imuState == ImuState::IMU_CALIBRATING) {
        computeOffsets();
    }
}

}  // namespace tap::communication::sensors::imu
