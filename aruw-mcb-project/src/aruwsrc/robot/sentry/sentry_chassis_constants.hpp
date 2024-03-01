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

#include "aruwsrc/control/chassis/new-chassis/swerve_wheel.hpp"
#include "aruwsrc/control/chassis/new-chassis/wheel.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"

namespace aruwsrc::sentry::chassis
{
// Distance from center of rotation to a swerve module
static constexpr float CENTER_TO_WHEELBASE_RADIUS = 0.205;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

// @todo generate this using the position offsets in the swerve module configs
/**
 * Calculated by solving for the pseudo-inverse of the following kinematics matrix
 *
 * 1, 0, -LF_Y
 * 0, 1,  LF_X
 * 1, 0, -RF_Y
 * 0, 1,  RF_X
 * 1, 0, -LB_Y
 * 0, 1,  LB_X
 * 1, 0, -RB_Y
 * 0, 1,  RB_X
 *
 * The above matrix gives the successive x, y components of the LF, RF, LB, and RB
 * module velocities given a [x, y, r] chassis velocity vector
 *
 */
constexpr float SWERVE_FORWARD_MATRIX[24]{
    0.25,      0.0,      0.25,     0.0,      0.25,      0.,        0.25,     0.0,
    0.0,       0.25,     0.0,      0.25,     0.0,       0.25,      0.0,      0.25,
    -0.862325, 0.862325, 0.862325, 0.862325, -0.862325, -0.862325, 0.862325, -0.862325};

// static constexpr SentryBeybladeCommand::SentryBeybladeConfig beybladeConfig{
//     .beybladeRotationalSpeedFractionOfMax = 0.45f,
//     .beybladeTranslationalSpeedMultiplier = 0.1f,
//     .beybladeRotationalSpeedMultiplierWhenTranslating = 0.7f,
//     .translationalSpeedThresholdMultiplierForRotationSpeedDecrease = 0.5f,
//     .beybladeRampRate = 45,
// };

// Using same azimuth and pid config for all swerve modules
aruwsrc::chassis::SwerveModuleConfig swerveConfig = {
    .driveMotorInverted = false,

};

modm::Pair<float, float> angular_power_frac_LUT[2] = {
        {0.0f, 0.2f},
        {M_PI_2, 0.75f},
    };

// todo: hopefullly these can live as constants here soon :)
aruwsrc::chassis::WheelConfig leftFrontSwerveConfig = {
    // .azimuthZeroOffset = 7888,

    .wheelPositionChassisRelativeX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = 0,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.076,
    .gearRatio = 23.0 / 12.0,
    .motorGearRatio = (1.0f / 19.0f),
    .velocityPidConfig = swerveConfig.drivePidConfig,
    // .driveMotorInverted = false,
    .inverted = true,
};

aruwsrc::chassis::SwerveAzimuthConfig leftFrontSwerveAzimuthConfig = {
    // .azimuthZeroOffset = 7888,
    .azimuthZeroOffset = 3753,
    .azimuthMotorGearing = 1.0,
    .azimuthPidConfig = swerveConfig.azimuthPidConfig,
    .angular_power_frac_LUT = {
        angular_power_frac_LUT[0],
        angular_power_frac_LUT[1],
    },
    .inverted = true};
aruwsrc::chassis::WheelConfig rightFrontSwerveConfig = {
    // .azimuthZeroOffset = 4452,

    .wheelPositionChassisRelativeX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = 0,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.076,
    .gearRatio = 23.0 / 12.0,
    .motorGearRatio = (1.0f / 19.0f),
    .velocityPidConfig = swerveConfig.drivePidConfig,
    // .driveMotorInverted = false,
    .inverted = true,
};

aruwsrc::chassis::SwerveAzimuthConfig rightFrontSwerveAzimuthConfig = {
    .azimuthZeroOffset = 356,
    .azimuthMotorGearing = 1.0,
    .azimuthPidConfig = swerveConfig.azimuthPidConfig,
    .angular_power_frac_LUT = {
        angular_power_frac_LUT[0],
        angular_power_frac_LUT[1],
    },
    .inverted = true};

aruwsrc::chassis::WheelConfig leftBackSwerveConfig = {
    // .azimuthZeroOffset = 7172,
    .wheelPositionChassisRelativeX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = 0,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.076,
    .gearRatio = 23.0 / 12.0,
    .motorGearRatio = (1.0f / 19.0f),
    .velocityPidConfig = swerveConfig.drivePidConfig,
    // .driveMotorInverted = false,
    .inverted = true,
};

aruwsrc::chassis::SwerveAzimuthConfig leftBackSwerveAzimuthConfig = {
    .azimuthZeroOffset = 3093,
    .azimuthMotorGearing = 1.0,
    .azimuthPidConfig = swerveConfig.azimuthPidConfig,
    .angular_power_frac_LUT = {
        angular_power_frac_LUT[0],
        angular_power_frac_LUT[1],
    },
    .inverted = true,
};

aruwsrc::chassis::WheelConfig rightBackSwerveConfig = {
    .wheelPositionChassisRelativeX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelPositionChassisRelativeY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .wheelOrientationChassisRelative = 0,
    .distFromCenterToWheel = CENTER_TO_WHEELBASE_RADIUS,
    .diameter = 0.076,
    .gearRatio = 23.0 / 12.0,
    .motorGearRatio = (1.0f / 19.0f),
    .velocityPidConfig = swerveConfig.drivePidConfig,
    .inverted = true,
};

aruwsrc::chassis::SwerveAzimuthConfig rightBackSwerveAzimuthConfig = {
    .azimuthZeroOffset = 3679,
    .azimuthMotorGearing = 1.0,
    .azimuthPidConfig = swerveConfig.azimuthPidConfig,
    .angular_power_frac_LUT = {
        angular_power_frac_LUT[0],
        angular_power_frac_LUT[1],
    },
    .inverted = true,
};

}  // namespace aruwsrc::sentry::chassis
#endif  // SENTRY_CHASSIS_CONSTANTS_HPP_
