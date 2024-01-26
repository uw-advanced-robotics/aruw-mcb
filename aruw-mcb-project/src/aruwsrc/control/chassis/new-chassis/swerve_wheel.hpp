/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SWERVE_WHEEL_HPP_
#define SWERVE_WHEEL_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "wheel.hpp"
#include "modm/container/pair.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "aruwsrc/algorithms/wheel.hpp"
#include "modm/math/geometry/angle.hpp"

#include "modm/math/filter/pid.hpp"
#include "../swerve_module_config.hpp"
using Motor = tap::motor::DjiMotor;
using SmoothPid = tap::algorithms::SmoothPid;
using SmoothPidConfig = tap::algorithms::SmoothPidConfig;

namespace aruwsrc
{
namespace chassis
{

struct SwerveAzimuthConfig
{
    int azimuthZeroOffset;
    float azimuthMotorGearing;
};


class SwerveWheel : public Wheel {

public:
SwerveWheel(
        Motor& driveMotor,
        Motor& azimuthMotor,
        WheelConfig& config,
        SwerveAzimuthConfig& azimuthConfig,
        SmoothPid drivePid,
        SmoothPid azimuthPid);
void executeWheelVelocity(float vx, float vy) override;
void refresh() override;
void initialize() override;
void setZeroRPM() override;
bool allMotorsOnline() const override;
float getDriveVelocity() const override;
float getDriveRPM() const override;
float getAngularVelocity() const;
float getAngle() const;

private:
    SwerveAzimuthConfig& azimuthConfig;
    Motor& driveMotor;
    Motor& azimuthMotor;

    SmoothPid drivePid;
    SmoothPid azimuthPid;
    
    const float rotationVectorX, rotationVectorY;
    float rotationSetpoint, speedSetpointRPM;  // pid setpoint, in radians and rpm respectively
    float preScaledSpeedSetpoint{0}, preScaledRotationSetpoint{0}, newRawRotationSetpointRadians,
        newRotationSetpointRadians, moveVectorX,
        
         moveVectorY;

    // handles unwrapping desired rotation and reversing module (in radians, will always be a
    // multiple of PI)
    float rotationOffset{0};

    //TODO: use wrappedFloat once merged in 
    inline float wrapAngle(float angle, float denomination)
    {
        return fmod(
            fmod(angle, denomination) + M_TWOPI,
            denomination);  // replace M_TWOPI with denomination? doesn't matter for its one use
                            // case currently
        // double fmod needed to ensure output is positive bc fmod can be negative
    } 

};
}
}

#endif 