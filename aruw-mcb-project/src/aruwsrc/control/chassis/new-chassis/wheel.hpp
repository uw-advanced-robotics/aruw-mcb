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
#include "tap/motor/dji_motor.hpp"

#include "modm/container/pair.hpp"
#include "modm/math/filter/pid.hpp"
using Motor = tap::motor::DjiMotor;
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
    float diameter;        // considering shoving these into DjiMotor in the future
    float gearRatio;       // considering shoving these into DjiMotor in the future
    float motorGearRatio;  // considering shoving these into DjiMotor in the future
    SmoothPidConfig& velocityPidConfig;
    bool isPowered = true;
    float maxWheelRPM;
};

class Wheel
{
public:
    /* Creates a wheel object using given motorId, x-direction distance from chassis center,
        y-direction distance from chassis center, wheel orientation, if wheel is powered
    */
    Wheel(Motor& driveMotor, WheelConfig& config);

    // Config parameters for the individual wheel
    const WheelConfig config;
    ;

    // PID used to control the driving motor
    const SmoothPid velocityPid;

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
    inline modm::Pair<float, float> calculateDesiredWheelVelocity(float vx, float vy, float vr)
    {
        CMSISMat<3, 1> chassisVel = tap::algorithms::CMSISMat<3, 1>({vx, vy, vr});
        CMSISMat<2, 1> wheelVel = distanceMat * chassisVel;
        return {wheelVel.data[0], wheelVel.data[1]};
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

    virtual void initialize();

    virtual void refresh();

    virtual void setZeroRPM();

    virtual bool allMotorsOnline() const;

    virtual float getDriveVelocity() const;

    virtual float getDriveRPM() const;

protected:
    // Motor that drives the wheel
    Motor& motor
        /// matrix containing distances from wheel to chassis center
        tap::algorithms::CMSISMat<2, 3>
            distanceMat = CMSISMat<2, 3>(
                {1,
                 0,
                 -config.wheelPositionChassisRelativeY,
                 0,
                 1,
                 config.wheelPositionChassisRelativeX});
};  // class Wheel
}  // namespace chassis
}  // namespace aruwsrc

#endif  // WHEEL_HPP_
