/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_CHASSIS_CONSTANTS_HPP_
#define SENTRY_CHASSIS_CONSTANTS_HPP_

#include "aruwsrc/control/chassis/swerve_module_config.hpp"

#include "sentry_beyblade_command.hpp"

namespace aruwsrc::sentry::chassis
{
// Distance from center of rotation to a swerve module
static constexpr float CENTER_TO_WHEELBASE_RADIUS = 0.230;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

// Initial position of the chassis in the field (meters)
static constexpr float INITIAL_CHASSIS_POSITION_X = 3.074f;
static constexpr float INITIAL_CHASSIS_POSITION_Y = 3.074f;

// @todo generate this using the position offsets in the swerve module configs
/**
 * Calculated by solving for the pseudo-inverse of the following kinematics matrix
 *
 * 1, 0, -RF_Y
 * 0, 1,  RF_X
 * 1, 0, -LB_Y
 * 0, 1,  LB_X
 *
 * The above matrix gives the successive x, y components of the LF, RF, LB, and RB
 * module velocities given a [x, y, r] chassis velocity vector
 */

// clang-format off
constexpr float HALF_SWERVE_FORWARD_MATRIX[12]{
    0.5,     0.0,      0.5,      0.5,      
    0.0,      0.5,     0.0,       0.25,    
    1.5371886548, 1.5371886548, -1.5371886548, -1.5371886548};
// clang-format on


tap::algorithms::SmoothPidConfig drivePidConfig = {
        .kp = 7.0f,
        .ki = 0.0f,
        .kd = -80.0f,
        .maxICumulative = 0.0f,
        .maxOutput = 16'000.0f,
        .tRDerivativeKalman = 100.0f,
        .tRProportionalKalman = 100.0f,
        .errDeadzone = 0.0f,
        .errorDerivativeFloor = 0.0f};

    tap::algorithms::SmoothPidConfig azimuthPidConfig = {
        .kp = 15000.0f,  // 10000.0f
        .ki = 0.0f,
        .kd = 0.0f,  // 12.0f
        .maxICumulative = 0.0f,
        .maxOutput = 16'000.0f,
        .errDeadzone = 0.0f,
        .errorDerivativeFloor = 0.0f,
    };

     

static constexpr SentryBeybladeCommand::SentryBeybladeConfig beybladeConfig{
    .beybladeRotationalSpeedFractionOfMax = 0.45f,
    .beybladeTranslationalSpeedMultiplier = 0.1f,
    .beybladeRotationalSpeedMultiplierWhenTranslating = 0.7f,
    .translationalSpeedThresholdMultiplierForRotationSpeedDecrease = 0.5f,
    .beybladeRampRate = 45,
};

// todo: hopefullly these can live as constants here soon :)
aruwsrc::chassis::SwerveModuleConfig rightFrontSwerveConfig = {
    // .azimuthZeroOffset = 4452,
    .azimuthZeroOffset = 7474 - (3 * DjiMotor::ENC_RESOLUTION / 8),
    .positionWithinChassisX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = false,
};

aruwsrc::chassis::SwerveAzimuthConfig rightFrontAzimuthConfig = {
    .azimuthZeroOffset = 3419 - (3 * DjiMotor::ENC_RESOLUTION / 8),
    .azimuthMotorGearing = 1.0f,
    .azimuthPidConfig = azimuthPidConfig,
    .inverted = false,
};

aruwsrc::chassis::SwerveAzimuthConfig leftBackAzimuthConfig = {
    .azimuthZeroOffset = 3419 - (3 * DjiMotor::ENC_RESOLUTION / 8),
    .azimuthMotorGearing = 1.0f,
    .azimuthPidConfig = azimuthPidConfig,
    .inverted = false,
};


aruwsrc::chassis::WheelConfig leftBackWheelConfig = {
    .wheelPositionChassisRelativeX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = 0,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.1016f,
    .gearRatio = 23.0f / 12.0f,
    .motorGearRatio = 1.0f / 19.0f,
    .velocityPidConfig = drivePidConfig,
    .inverted = false,
};

aruwsrc::chassis::WheelConfig rightFrontWheelConfig = {
    .wheelPositionChassisRelativeX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = 0,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.1016f,
    .gearRatio = 23.0f / 12.0f,
    .motorGearRatio = 1.0f / 19.0f,
    .velocityPidConfig = drivePidConfig,
    .inverted = false,
};

aruwsrc::chassis::WheelConfig rightOmniConfig = {
    .wheelPositionChassisRelativeX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = M_SQRT2 / 2.0f,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.102f,
    .gearRatio = 1,
    .motorGearRatio = 1,
    .velocityPidConfig = drivePidConfig,
    .isPowered = false,
    .inverted = false,
};

aruwsrc::chassis::WheelConfig leftOmniConfig = {
    .wheelPositionChassisRelativeX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = -M_SQRT2 / 2.0f,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.102f,
    .gearRatio = 1,
    .motorGearRatio = 1,
    .velocityPidConfig = drivePidConfig,
    .isPowered = false,
    .inverted = false,
};


aruwsrc::chassis::SwerveModuleConfig leftBackSwerveConfig = {
    // .azimuthZeroOffset = 7172,
    .azimuthZeroOffset = 3419 - (3 * DjiMotor::ENC_RESOLUTION / 8),
    .positionWithinChassisX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = false,
};

}  // namespace aruwsrc::sentry::chassis
#endif  // SENTRY_CHASSIS_CONSTANTS_HPP_
