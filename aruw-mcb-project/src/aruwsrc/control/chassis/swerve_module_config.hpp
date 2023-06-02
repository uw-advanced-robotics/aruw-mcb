/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SWERVE_MODULE_CONFIG_HPP_
#define SWERVE_MODULE_CONFIG_HPP_

#include "tap/algorithms/smooth_pid.hpp"

#include "modm/math/geometry/angle.hpp"
#include "modm/math/interpolation/linear.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

namespace aruwsrc::chassis
{
struct SwerveModuleConfig
{
    // @todo not really generic over future swerve designs?
    const float WHEEL_DIAMETER_M = 0.076f;
    const float WHEEL_CIRCUMFRENCE_M = WHEEL_DIAMETER_M * M_PI;

    // in encoder clicks, defines "forward" direction of the module
    // @todo why does this default?
    const int64_t azimuthZeroOffset = 0;

    tap::motor::MotorId driveMotorId = tap::motor::MOTOR1;
    tap::motor::MotorId azimuthMotorId = tap::motor::MOTOR5;

    // in meters, measured from center
    // @todo why defaullllltttssss T.T
    // @todo also, currently unused; see other todos in swerve drive and swerve module which suggest how to use them
    float positionWithinChassisX = 0.2f;
    float positionWithinChassisY = 0.2f;

    // @todo defaults and the number of fields makes it hard to make this auto-generated via constructor; resolve in another MR
    float distanceFromChassisCenter = 0.2f / M_SQRT2;

    // Whether any motor is inverted
    // @todo these should not be specified by default !!!
    const bool driveMotorInverted = true, azimuthMotorInverted = true;  // @todo doesn't quite work bc the motors are instantiated before the swerve modules
    // Gear ratios for motors
    const float driveMotorGearing = 23.0f / 12.0f, azimuthMotorGearing = 1;

    tap::algorithms::SmoothPidConfig drivePidConfig = {
        .kp = 8.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .maxICumulative = 0.0f,
        .maxOutput = 16'000.0f,
        .errDeadzone = 50.0f,
        .errorDerivativeFloor = 0.0f,
    };

    tap::algorithms::SmoothPidConfig azimuthPidConfig = {
        .kp = 15000.0f,//10000.0f
        .ki = 0.0f,
        .kd = 0.0f,//12.0f
        .maxICumulative = 0.0f,
        .maxOutput = 16'000.0f,
        .errDeadzone = 0.0f,
        .errorDerivativeFloor = 0.0f,
    };

    modm::Pair<float, float> ANGULAR_POWER_FRAC_LUT[2] = {
        {0.0f, 0.2f},
        {M_PI_2, 0.75f},
    };

    const float gearboxRatio = (1.0f / 19.0f);  // @todo no idea if this is right
};

static SwerveModuleConfig DEFAULT_SWERVE_CONFIG;

}  // namespace aruwsrc::chassis
#endif  // SWERVE_MODULE_CONFIG_HPP_