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
static constexpr float CENTER_TO_WHEELBASE_RADIUS = 0.205;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

// Initial position of the chassis in the field (meters)
static constexpr float INITIAL_CHASSIS_POSITION_X = 3.074f;
static constexpr float INITIAL_CHASSIS_POSITION_Y = 3.074f;

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

static constexpr SentryBeybladeCommand::SentryBeybladeConfig beybladeConfig{
    .beybladeRotationalSpeedFractionOfMax = 0.45f,
    .beybladeTranslationalSpeedMultiplier = 0.1f,
    .beybladeRotationalSpeedMultiplierWhenTranslating = 0.7f,
    .translationalSpeedThresholdMultiplierForRotationSpeedDecrease = 0.5f,
    .beybladeRampRate = 45,
};

// todo: hopefullly these can live as constants here soon :)
aruwsrc::chassis::SwerveModuleConfig leftFrontSwerveConfig = {
    // .azimuthZeroOffset = 7888,
    .azimuthZeroOffset = 3753,
    .positionWithinChassisX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

aruwsrc::chassis::SwerveModuleConfig rightFrontSwerveConfig = {
    // .azimuthZeroOffset = 4452,
    .azimuthZeroOffset = 356,
    .positionWithinChassisX = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

aruwsrc::chassis::SwerveModuleConfig leftBackSwerveConfig = {
    // .azimuthZeroOffset = 7172,
    .azimuthZeroOffset = 3093,
    .positionWithinChassisX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

aruwsrc::chassis::SwerveModuleConfig rightBackSwerveConfig = {
    // .azimuthZeroOffset = 7878,
    .azimuthZeroOffset = 3679,
    .positionWithinChassisX = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    .positionWithinChassisY = -CENTER_TO_WHEELBASE_RADIUS / M_SQRT2,
    // .driveMotorInverted = false,
    .driveMotorInverted = true,
};

}  // namespace aruwsrc::sentry::chassis
#endif  // SENTRY_CHASSIS_CONSTANTS_HPP_
