#ifndef TAPROOT_ABSTRACT_IMU_HPP_
#define TAPROOT_ABSTRACT_IMU_HPP_

#include "tap/algorithms/MahonyAHRS.h"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/communication/sensors/imu_heater/imu_heater.hpp"

namespace tap::communication::sensors::imu {

class AbstractIMU : public ImuInterface {
public:
    explicit AbstractIMU(tap::Drivers *drivers)
        : drivers(drivers), imuHeater(drivers) {}

    virtual ~AbstractIMU() = default;

    virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);
    virtual void requestCalibration();
    virtual void periodicIMUUpdate();
    virtual bool read() = 0;  

    virtual float getYaw() final { return mahonyAlgorithm.getYaw(); }
    virtual float getPitch() final { return mahonyAlgorithm.getPitch(); }
    virtual float getRoll() final { return mahonyAlgorithm.getRoll(); }

    virtual ImuState getImuState() const final { return imuState; }

    virtual float getAx() = 0;
    virtual float getAy() = 0;
    virtual float getAz() = 0;

    virtual float getGx() = 0;
    virtual float getGy() = 0;
    virtual float getGz() = 0;

    virtual float getTemp() = 0;

protected:
    virtual void resetOffsets() = 0;
    virtual void computeOffsets() = 0;

    tap::Drivers *drivers;
    Mahony mahonyAlgorithm;
    imu_heater::ImuHeater imuHeater;
    ImuState imuState = ImuState::IMU_NOT_CONNECTED;
    int calibrationSample = 0;
};

}  // namespace tap::communication::sensors::imu

#endif  // TAPROOT_ABSTRACT_IMU_HPP_
