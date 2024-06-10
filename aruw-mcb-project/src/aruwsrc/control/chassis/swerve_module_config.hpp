/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
    const float WHEEL_DIAMETER_M = 0.1016f;
    const float WHEEL_CIRCUMFRENCE_M = WHEEL_DIAMETER_M * M_PI;

    // in encoder clicks, defines "forward" direction of the module
    // @todo why does this default?
    const int64_t azimuthZeroOffset = 0;

    tap::motor::MotorId driveMotorId = tap::motor::MOTOR1;
    tap::motor::MotorId azimuthMotorId = tap::motor::MOTOR5;

    // in meters, measured from center
    float positionWithinChassisX = 0.2f;
    float positionWithinChassisY = 0.2f;

    // @todo defaults and the number of fields makes it hard to make this auto-generated via
    // constructor; resolve in another MR
    float distanceFromChassisCenter = 0.2f / M_SQRT2;

    // Whether any motor is inverted
    const bool driveMotorInverted;
    const bool azimuthMotorInverted =
        true;  // @todo doesn't quite make sense to put here bc the motors are instantiated before
               // the swerve modules (see main sentry control)
    // Gear ratios for motors
    const float driveMotorGearing = 23.0f / 12.0f;
    const float azimuthMotorGearing = 1;

    tap::algorithms::SmoothPidConfig drivePidConfig = {
        .kp = 10.0f,
        .ki = 0.8f,
        .kd = 0.0f,
        .maxICumulative = 1.0f,
        .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
        .tRDerivativeKalman = 100.0f,
        .tRProportionalKalman = 500.0f,
        .errDeadzone = 300.0f,  // for motor backlash
        .errorDerivativeFloor = 0.0f};

    tap::algorithms::SmoothPidConfig azimuthPidConfig = {
        .kp = 62'000.0f,
        .ki = 700.0f,
        .kd = 0.0f,
        .maxICumulative = 700.0f,
        .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_GM6020,
        .errDeadzone = 0.0f,
        .errorDerivativeFloor = 0.0f,
    };

    modm::Pair<float, float> ANGULAR_POWER_FRAC_LUT[2] = {
        {0.0f, 0.2f},
        {M_PI_2, 0.75f},
    };

    const float gearboxRatio = (1.0f / 19.0f);
};

}  // namespace aruwsrc::chassis
#endif  // SWERVE_MODULE_CONFIG_HPP_