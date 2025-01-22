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

    inline float getAx()  override { return imuData.accG[ImuData::X]; }
    inline float getAy()  override { return imuData.accG[ImuData::Y]; }
    inline float getAz()  override { return imuData.accG[ImuData::Z]; }

    inline float getGx()  override { return imuData.gyroDegPerSec[ImuData::X]; }
    inline float getGy()  override { return imuData.gyroDegPerSec[ImuData::Y]; }
    inline float getGz()  override { return imuData.gyroDegPerSec[ImuData::Z]; }

    inline float getTemp() override { return imuData.temperature; }

    struct ImuData
    {
        enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2,
        };

        float accRaw[3] = {0};
        float gyroRaw[3] = {0};
        float accOffsetRaw[3] = {0};
        float gyroOffsetRaw[3] = {0};
        float accG[3] = {0};
        float gyroDegPerSec[3] = {0};

        float temperature = 0;
    };

protected:
    virtual void resetOffsets() = 0;
    virtual void computeOffsets() = 0;

    virtual float getAccelerationSenstivity() = 0;

    tap::Drivers *drivers;

    Mahony mahonyAlgorithm;
    
    ImuState imuState = ImuState::IMU_NOT_CONNECTED;
    int calibrationSample = 0;
    
    ImuData imuData;
};

}  // namespace tap::communication::sensors::imu

#endif  // TAPROOT_ABSTRACT_IMU_HPP_
