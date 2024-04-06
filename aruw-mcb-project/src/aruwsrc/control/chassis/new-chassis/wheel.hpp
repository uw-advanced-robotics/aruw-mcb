/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef WHEEL_HPP_
#define WHEEL_HPP_

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/smooth_pid.hpp"

#include "modm/container/pair.hpp"
#include "modm/math/filter/pid.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
#include "tap/motor/dji_motor.hpp"
using Motor = tap::motor::DjiMotor;
#endif

using SmoothPid = tap::algorithms::SmoothPid;
using SmoothPidConfig = tap::algorithms::SmoothPidConfig;
using namespace tap::algorithms;
namespace aruwsrc
{
namespace chassis
{
// create a struct with wheel pose/radius/orientation
struct WheelConfig
{
    float wheelPositionChassisRelativeX;
    float wheelPositionChassisRelativeY;
    float wheelOrientationChassisRelative;
    // Check that this is the correct value for any arbitrary wheel placement
    // In a configuration with the wheels at non standrd distances to the center,
    // this should the maximum of all four.
    float distFromCenterToWheel;
    float diameter;   // considering shoving these into DjiMotor in the future
    float gearRatio;  // considering shoving these into DjiMotor in the future
    // considering combining gearRatio and motorGearRatio into one
    float motorGearRatio;  // considering shoving these into DjiMotor in the future
    const SmoothPidConfig& velocityPidConfig;
    bool isPowered = true;
    float maxWheelRPM = 1000;
    bool inverted;
};

class Wheel
{
public:
    /* Creates a wheel object using given motorId, x-direction distance from chassis center,
        y-direction distance from chassis center, wheel orientation, if wheel is powered
    */
    Wheel(const WheelConfig& config) : config(config)
    {
        distanceMat = CMSISMat<2, 3>(
            {1,
             0,
             -config.wheelPositionChassisRelativeY,
             0,
             1,
             config.wheelPositionChassisRelativeX});
        wheelOrientationMat = CMSISMat<2, 2>(
            {cos(config.wheelOrientationChassisRelative),
             -sin(config.wheelOrientationChassisRelative),
             sin(config.wheelOrientationChassisRelative),
             cos(config.wheelOrientationChassisRelative)});
        wheelOrientationMat = wheelOrientationMat.inverse();
    }

    // Config parameters for the individual wheel
    const WheelConfig config;

    /**
     * Calculates desired x and y velocity of the wheel based on passed in x, y, and r
     * components of the chassis velocity
     *
     * @param[in] vx The desired velocity of the chassis to move in the x direction in m/s
     * @param[in] vy The desired velocity of the chassis to move in the y direction in m/s
     * @param[in] vr The desired rotational velocity of the chassis in rpm
     *
     * @return a float Pair with the first value containing the desired velocity of the wheel
     *         in the x direction and the second value containing the desired velocity
     *         of the wheel in the y direction. Units: m/s. Might change type later???
     */
    inline modm::Pair<float, float> calculateDesiredWheelVelocity(
        float vx,
        float vy,
        float vr)  // chassis in mps, mps, rad/s
    {
        CMSISMat<3, 1> chassisVel = CMSISMat<3, 1>({vx, vy, vr});
        CMSISMat<2, 1> wheelVel = wheelOrientationMat * distanceMat * chassisVel;
        return {
            wheelVel.data[0],
            wheelVel.data[1]};  // wheel speed in mps if lx and ly are meters in chassis frame
                                // common interface -- multiply resulting matrix with the sin/cos
                                // matrices too for
                                // wheel relative speeds
    }

    /**
     * Updates the desired wheel RPM based on passed in x and y components of desired
     * wheel velocity
     * @param[in] vx The desired velocity of the wheel to move in the x direction in m/s
     * @param[in] vy The desired velocity of the wheel to move in the y direction in m/s
     */
    virtual void executeWheelVelocity(float vx, float vy) = 0;
    inline float mpsToRpm(float mps) const
    {
        return (mps / (config.diameter * M_PI)) / config.motorGearRatio * 60.0f / config.gearRatio;
    }

    inline float rpmToMps(float rpm) const
    {
        return rpm * config.motorGearRatio / 60.0f * config.gearRatio * (config.diameter * M_PI);
    }

    inline float rpmToRadPerS(float rpm) const
    {
        return rpm * M_2_PI / 60.0f * config.motorGearRatio * config.gearRatio;
    }

    virtual void initialize() = 0;

    virtual void refresh() = 0;

    virtual void setZeroRPM() = 0;

    virtual void limitPower(float powerLimitFrac) = 0;

    virtual bool allMotorsOnline() const = 0;

    virtual float getDriveVelocity() const = 0;

    virtual float getDriveRPM() const = 0;

    virtual int getNumMotors() const = 0;

protected:
    /// matrix containing distances from wheel to chassis center
    tap::algorithms::CMSISMat<2, 3> distanceMat;
    tap::algorithms::CMSISMat<2, 2> wheelOrientationMat;
};  // class Wheel
}  // namespace chassis
}  // namespace aruwsrc

#endif  // WHEEL_HPP_
