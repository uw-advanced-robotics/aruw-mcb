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

#ifndef DRONE_IMU_HPP_
#define DRONE_IMU_HPP_

#include "tap/algorithms/filter/butterworth.hpp"

#include "aruwsrc/communication/sensors/imu/ism330/ism330.hpp"

namespace aruwsrc::drone
{
class DroneIMU : public aruwsrc::communication::sensors::imu::ism330::ISM330
{
public:
    DroneIMU();

    void initialize(float sampleFrequency, float mahonyKp, float mahonyKi) override;

    void periodicIMUUpdate() override;

    float getYaw() const override;
    float getPitch() const override;
    float getRoll() const override;

    float getQ0() const { return q0; }
    float getQ1() const { return q1; }
    float getQ2() const { return q2; }
    float getQ3() const { return q3; }

private:
    using ImuOutputFilter = tap::algorithms::filter::DiscreteFilter<3, float>;

    void resetQuaternion();
    void updateQuaternion(float gx, float gy, float gz, float ax, float ay, float az);
    void filterQuaternionOutput();
    void computeEulerAngles();

    float twoKp = 0.0f;
    float twoKi = 0.0f;
    float invSampleFreq = 0.0f;

    float rawQ0 = 1.0f;
    float rawQ1 = 0.0f;
    float rawQ2 = 0.0f;
    float rawQ3 = 0.0f;

    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    float integralFBx = 0.0f;
    float integralFBy = 0.0f;
    float integralFBz = 0.0f;

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    ImuOutputFilter q0Filter;
    ImuOutputFilter q1Filter;
    ImuOutputFilter q2Filter;
    ImuOutputFilter q3Filter;
};
}  // namespace aruwsrc::drone

#endif  // DRONE_IMU_HPP_
