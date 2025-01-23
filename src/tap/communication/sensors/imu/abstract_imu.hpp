#ifndef TAPROOT_ABSTRACT_IMU_HPP_
#define TAPROOT_ABSTRACT_IMU_HPP_

#include "tap/algorithms/MahonyAHRS.h"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/transforms/orientation.hpp"


namespace tap
{
class Drivers;
}
using tap::algorithms::transforms::Transform;
using tap::algorithms::transforms::Orientation;

namespace tap::communication::sensors::imu {

class AbstractIMU : public ImuInterface {
public:
    explicit AbstractIMU(tap::Drivers *drivers)
    : drivers(drivers), mountingTransform(Transform::identity()){}

    AbstractIMU(const Transform& mountingTransform = Transform(Transform::identity()));
    void setMountingTransform(const Transform& transform);

    virtual ~AbstractIMU() = default;

    virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);

    /**
     * When this function is called, the bmi088 enters a calibration state during which time,
     * gyro/accel calibration offsets will be computed and the mahony algorithm reset. When
     * calibrating, angle, accelerometer, and gyroscope values will return 0. When calibrating
     * the BMI088 should be level, otherwise the IMU will be calibrated incorrectly.
     */
    virtual void requestCalibration();

    /**
     * Call this function at same rate as intialized sample frequency.
     * Performs the mahony AHRS algorithm to compute pitch/roll/yaw.
     */
    virtual void periodicIMUUpdate();

    virtual bool read() = 0;  



    /**
     * Returns the state of the IMU. Can be not connected, connected but not calibrated, or
     * calibrated. When not connected, IMU data will be garbage. When not calibrated, IMU data is
     * valid but the computed yaw angle data will drift. When calibrating, the IMU data is invalid.
     * When calibrated, the IMU data is valid and assuming proper calibration the IMU data should
     * not drift.
     *
     * To be safe, whenever you call the functions below, call this function to ensure
     * the data you are about to receive is not garbage.
     */
    virtual ImuState getImuState() const final { return imuState; }

    inline float getAx()  override { return imuData.accG[ImuData::X]; }
    inline float getAy()  override { return imuData.accG[ImuData::Y]; }
    inline float getAz()  override { return imuData.accG[ImuData::Z]; }

    inline float getGx()  override { return imuData.gyroDegPerSec[ImuData::X]; }
    inline float getGy()  override { return imuData.gyroDegPerSec[ImuData::Y]; }
    inline float getGz()  override { return imuData.gyroDegPerSec[ImuData::Z]; }

    inline float getTemp() override { return imuData.temperature; }

    virtual float getYaw() override { return mahonyAlgorithm.getYaw(); }
    virtual float getPitch() override { return mahonyAlgorithm.getPitch(); }
    virtual float getRoll() override { return mahonyAlgorithm.getRoll(); }

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

    void setCalibrationSamples(int sampleCount) {
        offsetSampleCount = sampleCount;
    }

protected:
    void resetOffsets();
    void computeOffsets();

    virtual inline float getAccelerationSensitivity() = 0;

    tap::Drivers *drivers;
    tap::algorithms::transforms::Transform mountingTransform;

    Mahony mahonyAlgorithm;
    
    ImuState imuState = ImuState::IMU_NOT_CONNECTED;
    int calibrationSample = 0;
    int offsetSampleCount = 1000;
    
    ImuData imuData;

    tap::arch::MicroTimeout readTimeout;
    
    uint32_t prevIMUDataReceivedTime = 0;
};

}  // namespace tap::communication::sensors::imu

#endif  // TAPROOT_ABSTRACT_IMU_HPP_
