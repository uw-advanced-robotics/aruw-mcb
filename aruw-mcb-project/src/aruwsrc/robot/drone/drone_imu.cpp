/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_imu.hpp"

#include <cmath>

#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::drone
{
namespace
{
static constexpr float IMU_OUTPUT_LOWPASS_CUTOFF_HZ = 100.0f;
static constexpr float DEFAULT_SAMPLE_FREQUENCY_HZ = 500.0f;

tap::algorithms::filter::Coefficients<3, float> makeImuOutputLowpassCoefficients(
    float sampleFrequency)
{
    return tap::algorithms::filter::butterworth<2, tap::algorithms::filter::LOWPASS>(
        2.0f * static_cast<float>(M_PI) * IMU_OUTPUT_LOWPASS_CUTOFF_HZ,
        1.0f / sampleFrequency);
}
}  // namespace

DroneIMU::DroneIMU()
    : q0Filter(makeImuOutputLowpassCoefficients(DEFAULT_SAMPLE_FREQUENCY_HZ)),
      q1Filter(makeImuOutputLowpassCoefficients(DEFAULT_SAMPLE_FREQUENCY_HZ)),
      q2Filter(makeImuOutputLowpassCoefficients(DEFAULT_SAMPLE_FREQUENCY_HZ)),
      q3Filter(makeImuOutputLowpassCoefficients(DEFAULT_SAMPLE_FREQUENCY_HZ))
{
    resetQuaternion();
}

void DroneIMU::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    ISM330::initialize(sampleFrequency, mahonyKp, mahonyKi);
    const auto imuOutputLowpassCoefficients = makeImuOutputLowpassCoefficients(sampleFrequency);
    q0Filter.setCoefficients(imuOutputLowpassCoefficients);
    q1Filter.setCoefficients(imuOutputLowpassCoefficients);
    q2Filter.setCoefficients(imuOutputLowpassCoefficients);
    q3Filter.setCoefficients(imuOutputLowpassCoefficients);
    invSampleFreq = 1.0f / sampleFrequency;
    twoKp = 2.0f * mahonyKp;
    twoKi = 2.0f * mahonyKi;
    resetQuaternion();
}

void DroneIMU::periodicIMUUpdate()
{
    const ImuState stateBeforeUpdate = imuState;
    updateImuMeasurement();

    if (imuState == ImuState::IMU_CALIBRATING)
    {
        computeOffsets();
        if (stateBeforeUpdate == ImuState::IMU_CALIBRATING && imuState == ImuState::IMU_CALIBRATED)
        {
            resetQuaternion();
        }
        return;
    }

    updateQuaternion(
        imuData.gyroRadPerSec.x(),
        imuData.gyroRadPerSec.y(),
        imuData.gyroRadPerSec.z(),
        imuData.accG.x(),
        imuData.accG.y(),
        imuData.accG.z());
}

float DroneIMU::getYaw() const { return fmodf(yaw + M_TWOPI, M_TWOPI); }

float DroneIMU::getPitch() const { return pitch; }

float DroneIMU::getRoll() const { return roll; }

void DroneIMU::resetQuaternion()
{
    rawQ0 = 1.0f;
    rawQ1 = 0.0f;
    rawQ2 = 0.0f;
    rawQ3 = 0.0f;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    q0Filter.setSteadyState(q0);
    q1Filter.setSteadyState(q1);
    q2Filter.setSteadyState(q2);
    q3Filter.setSteadyState(q3);
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
}

void DroneIMU::updateQuaternion(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float qa;
    float qb;
    float qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        const float halfvx = rawQ1 * rawQ3 - rawQ0 * rawQ2;
        const float halfvy = rawQ0 * rawQ1 + rawQ2 * rawQ3;
        const float halfvz = rawQ0 * rawQ0 - 0.5f + rawQ3 * rawQ3;

        const float halfex = ay * halfvz - az * halfvy;
        const float halfey = az * halfvx - ax * halfvz;
        const float halfez = ax * halfvy - ay * halfvx;

        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invSampleFreq;
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    gx *= 0.5f * invSampleFreq;
    gy *= 0.5f * invSampleFreq;
    gz *= 0.5f * invSampleFreq;

    qa = rawQ0;
    qb = rawQ1;
    qc = rawQ2;
    rawQ0 += -qb * gx - qc * gy - rawQ3 * gz;
    rawQ1 += qa * gx + qc * gz - rawQ3 * gy;
    rawQ2 += qa * gy - qb * gz + rawQ3 * gx;
    rawQ3 += qa * gz + qb * gy - qc * gx;

    recipNorm = 1.0f / sqrtf(rawQ0 * rawQ0 + rawQ1 * rawQ1 + rawQ2 * rawQ2 + rawQ3 * rawQ3);
    rawQ0 *= recipNorm;
    rawQ1 *= recipNorm;
    rawQ2 *= recipNorm;
    rawQ3 *= recipNorm;

    filterQuaternionOutput();
    computeEulerAngles();
}

void DroneIMU::filterQuaternionOutput()
{
    q0 = q0Filter.filterData(rawQ0);
    q1 = q1Filter.filterData(rawQ1);
    q2 = q2Filter.filterData(rawQ2);
    q3 = q3Filter.filterData(rawQ3);

    const float recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void DroneIMU::computeEulerAngles()
{
    roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
    yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
}
}  // namespace aruwsrc::drone
