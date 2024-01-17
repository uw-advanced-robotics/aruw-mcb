/*
 * Copyright (c) 2021-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
// #include "sentry_beyblade_command.hpp"  //TODO: uncomment this stuff when command is created

// Do not include this file directly: use chassis_constants.hpp instead.
#ifndef CHASSIS_CONSTANTS_HPP_
#error "Do not include this file directly! Use chassis_constants.hpp instead."
#endif

using namespace aruwsrc::chassis;
//{
// static constexpr float BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER = 0.6f;

/**
 * "Maps max power (in Watts) to max chassis wheel speed (RPM).
 *
 * Since the engineer has no power limiting, this lookup table doesn't matter much, just set some
 * high values."
 * TODO: this may have actually been used in comp...? get actual values
 */
// static constexpr modm::Pair<int, float> CHASSIS_POWER_TO_MAX_SPEED_LUT[] = {{1, 8'000}, {1,
// 8'000}};

// static modm::interpolation::Linear<modm::Pair<int, float>> CHASSIS_POWER_TO_SPEED_INTERPOLATOR(
//     CHASSIS_POWER_TO_MAX_SPEED_LUT,
//     MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT));

// }  // namespace aruwsrc::chassis

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

// static constexpr SentryBeybladeCommand::SentryBeybladeConfig beybladeConfig
// {
//     .beybladeRotationalSpeedFractionOfMax = 0.45f,
//     .beybladeTranslationalSpeedMultiplier = 0.1f,
//     .beybladeRotationalSpeedMultiplierWhenTranslating = 0.7f,
//     .translationalSpeedThresholdMultiplierForRotationSpeedDecrease = 0.5f,
//     .beybladeRampRate = 45,
// };

// todo: hopefullly these can live as constants here soon :)
SwerveModuleConfig leftFrontSwerveConfig = {
    // .azimuthZeroOffset = 7888,
    .azimuthZeroOffset = 3753,
    .positionWithinChassisX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

SwerveModuleConfig rightFrontSwerveConfig = {
    // .azimuthZeroOffset = 4452,
    .azimuthZeroOffset = 356,
    .positionWithinChassisX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

SwerveModuleConfig leftBackSwerveConfig = {
    // .azimuthZeroOffset = 7172,
    .azimuthZeroOffset = 3093,
    .positionWithinChassisX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

SwerveModuleConfig rightBackSwerveConfig = {
    // .azimuthZeroOffset = 7878,
    .azimuthZeroOffset = 3679,
    .positionWithinChassisX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

}  // namespace aruwsrc::sentry::chassis
#endif  // SENTRY_CHASSIS_CONSTANTS_HPP_
