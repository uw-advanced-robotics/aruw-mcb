#ifndef IMU_HEADING_FUSION_HPP_
#define IMU_HEADING_FUSION_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/linear_interpolation_contiguous.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::algorithms
{
class ImuHeadingFusion
{
public:
    ImuHeadingFusion(tap::Drivers *drivers);

    void initialize();

    void run();

    float getYaw() const { return yawFiltered; }

private:
    /**
     * Gyroscope z rotational velocity, in degrees/second. When the gyroscope rotational velocity is
     * above this value, the mpu6500 is used exclusively. When the velocity is below this value,
     * a combination of the bno055 and mpu6500 is used.
     */
    static constexpr float MAX_GZ_FOR_FULL_MPU_RELIANCE = 10.0f;

    /**
     * The average of the absolute mpu6500 gyroscope readings in the z axis, rounded up.
     */
    static constexpr float MPU6500_RESTING_MAX_GZ = 0.5f;

    tap::Drivers *drivers;

    bool imusInitialized = false;

    float mpu6500YawOffset = 0.0f;
    float bno055YawOffset = 0.0f;
    /**
     * @note We must invert the bno055 so its direction matches that of the mpu6500.
     */
    tap::algorithms::ContiguousFloat bno055InvertedPrevious;
    tap::algorithms::LinearInterpolationContiguous bno055LinearlyInterpolated;
    float yawFiltered = 0.0f;

    void resetMpuCalibrationOffset(float bnoYaw, float mpuYaw);
    void resetBnoCalibrationOffset(float bnoYaw, float mpuYaw);
};
}  // namespace aruwsrc::algorithms

#endif  // IMU_HEADING_FUSION_HPP_
