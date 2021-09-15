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

#include "imu_heading_fusion.hpp"

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

using namespace tap::algorithms;
using namespace tap::arch::clock;

namespace aruwsrc::algorithms
{
ImuHeadingFusion::ImuHeadingFusion(tap::Drivers *drivers)
    : drivers(drivers),
      bno055InvertedPrevious(0, 0, 360),
      bno055LinearlyInterpolated(0, 360)
{
}

void ImuHeadingFusion::initialize()
{
    imusInitialized = false;
    mpu6500YawOffset = 0.0f;
    bno055YawOffset = 0.0f;
    bno055InvertedPrevious.setValue(0.0f);
    yawFiltered = 0.0f;
    bno055LinearlyInterpolated.reset(0.0f, getTimeMilliseconds());
}

static inline float getOffsetYawValue(float rawYaw, float calibrationOffset)
{
    return ContiguousFloat(rawYaw + calibrationOffset, 0, 360).getValue();
}

void ImuHeadingFusion::run()
{
    if (drivers->mpu6500.initialized() && drivers->bno055InterfaceFusion.isReady())
    {
        // Perform linear interpolation of the bno055 data, which is read at 100 Hz.
        ContiguousFloat newYaw(360.0f - drivers->bno055InterfaceFusion.getYaw(), 0.0f, 360.0f);
        if (drivers->bno055InterfaceFusion.isNewDataReady())
        {
            drivers->bno055InterfaceFusion.setNewDataNotReady();
            if (!imusInitialized)
            {
                bno055LinearlyInterpolated.reset(newYaw.getValue(), getTimeMilliseconds());
            }
            else
            {
                bno055LinearlyInterpolated.update(newYaw.getValue(), getTimeMilliseconds());
                bno055InvertedPrevious.setValue(newYaw.getValue());
            }
        }

        // Get BNO055 and MPU6500 yaw values
        const float bno055Yaw =
            bno055LinearlyInterpolated.getInterpolatedValue(getTimeMilliseconds());
        const float mpu6500Yaw = drivers->mpu6500.getYaw();

        // Reset calib offsets when first starting
        if (!imusInitialized)
        {
            imusInitialized = true;
            resetMpuCalibrationOffset(bno055Yaw, mpu6500Yaw);
            resetBnoCalibrationOffset(bno055Yaw, mpu6500Yaw);
        }

        /*
         * Get gyroscope angular velocity, round to 0 if within the resting
         * average.
         */
        float gz = fabsf(drivers->mpu6500.getGz());
        if (gz < MPU6500_RESTING_MAX_GZ)
        {
            gz = 0.0f;
        }

        /*
         * Ratio between [0, 1] indicating which IMU to rely on at some particular
         * point in time. If 0, rely on the bno, if 1 rely on the mpu.
         */
        const float mpuToBnoRatio = limitVal(gz / MAX_GZ_FOR_FULL_MPU_RELIANCE, 0.0f, 1.0f);

        float bno055YawWithOffset = 0.0f;
        float mpu6500YawWithOffset = 0.0f;

        /*
         * Reset offsets of bno or mpu if either are not being used to compute
         * the final yaw.
         */
        if (compareFloatClose(mpuToBnoRatio, 0.0f, 1E-5))
        {
            // Using bno055 completely
            bno055YawWithOffset = getOffsetYawValue(bno055Yaw, bno055YawOffset);
            resetMpuCalibrationOffset(bno055YawWithOffset, mpu6500Yaw);
            mpu6500YawWithOffset = getOffsetYawValue(mpu6500Yaw, mpu6500YawOffset);
        }
        else if (compareFloatClose(mpuToBnoRatio, 1.0f, 1E-5))
        {
            // Using mpu6500 completely
            mpu6500YawWithOffset = getOffsetYawValue(mpu6500Yaw, mpu6500YawOffset);
            resetBnoCalibrationOffset(bno055Yaw, mpu6500YawWithOffset);
            bno055YawWithOffset = getOffsetYawValue(bno055Yaw, bno055YawOffset);
        }
        else
        {
            // Use both mpu6500/bno055, don't reset calibration offsets
            bno055YawWithOffset = getOffsetYawValue(bno055Yaw, bno055YawOffset);
            mpu6500YawWithOffset = getOffsetYawValue(mpu6500Yaw, mpu6500YawOffset);
        }

        // Compute weighted average based on mpuToBnoRatio
        yawFiltered =
            mpuToBnoRatio * mpu6500YawWithOffset + (1.0f - mpuToBnoRatio) * bno055YawWithOffset;
    }
}
}  // namespace aruwsrc::algorithms
