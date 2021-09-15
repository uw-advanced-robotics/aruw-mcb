/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

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
/**
 * An object whose purpose is to combine data from a BNO055 and MPU6500
 * to improve reduce the yaw drift when stationary.
 */
class ImuHeadingFusion
{
public:
    ImuHeadingFusion(tap::Drivers *drivers);

    void initialize();

    /**
     * Function meant to be called at 500 Hz, the rate at which the mpu6500's
     * `periodicIMUUpdate` function should be called.
     */
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

    inline void resetMpuCalibrationOffset(float bnoYaw, float mpuYaw)
    {
        mpu6500YawOffset = -mpuYaw + bnoYaw;
    }
    inline void resetBnoCalibrationOffset(float bnoYaw, float mpuYaw)
    {
        bno055YawOffset = mpuYaw - bnoYaw;
    }
};
}  // namespace aruwsrc::algorithms

#endif  // IMU_HEADING_FUSION_HPP_
