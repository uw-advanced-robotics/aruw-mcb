/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FLYWHEEL_TESTING_AGITATOR_CONSTANTS_HPP_
#define FLYWHEEL_TESTING_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/velocity_agitator_subsystem_config.hpp"
#include "modm/math/geometry.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

namespace aruwsrc::control::agitator::constants
{
static constexpr float DESIRED_LOAD_TIME_S = 0.25f;
static constexpr float KICKER_DESIRED_LOAD_TIME_S = 0.1f;
static constexpr float KICKER_FIRE_DISTANCE = M_TWOPI * 0.5f;
static constexpr float KICKER_FIRE_TIME_S = 0.075f;

// PID terms for the hero kicker
static constexpr tap::algorithms::SmoothPidConfig KICKER_PID_CONFIG = {
    .kp = 5'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16000.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr aruwsrc::control::agitator::VelocityAgitatorSubsystemConfig
    KICKER_AGITATOR_CONFIG = {
        .gearRatio = 1.0f / 36.0f,
        .agitatorMotorId = tap::motor::MOTOR2,
        .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
        .isAgitatorInverted = false,
        .jammingVelocityDifference = 0,
        .jammingTime = 0,
        .jamLogicEnabled = false,
        .velocityPIDFeedForwardGain = 0,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config
    KICKER_LOAD_AGITATOR_ROTATE_CONFIG = {
        .targetIntegralChange = M_TWOPI / 32.0f,
        .desiredSetpoint = (M_TWOPI / 32.0f) / KICKER_DESIRED_LOAD_TIME_S,
        .integralSetpointTolerance = 0,  /// This tolerance can be 0 since the command considers
                                         /// itself done when the integral setpoint is >= initial
                                         /// integral + targetIntegralChange -
                                         /// integralSetpointTolerance. Thus, it is reasonable
                                         /// for this to be 0.
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config
    KICKER_SHOOT_AGITATOR_ROTATE_CONFIG = {
        .targetIntegralChange = KICKER_FIRE_DISTANCE,
        .desiredSetpoint = KICKER_FIRE_DISTANCE / KICKER_FIRE_TIME_S,
        .integralSetpointTolerance = 0,  /// This tolerance can be 0 since the command considers
                                         /// itself done when the integral setpoint is >= initial
                                         /// integral + targetIntegralChange -
                                         /// integralSetpointTolerance. Thus, it is reasonable
                                         /// for this to be 0.
};

/// How much extra heat must be available beyond how much it takes to fire the next shot
static constexpr uint16_t HEAT_LIMIT_BUFFER = 0;
}  // namespace aruwsrc::control::agitator::constants

#endif  // FLYWHEEL_TESTING_AGITATOR_CONSTANTS_HPP_
