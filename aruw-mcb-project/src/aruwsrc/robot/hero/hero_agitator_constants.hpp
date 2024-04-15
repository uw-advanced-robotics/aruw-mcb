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

#ifndef HERO_AGITATOR_CONSTANTS_HPP_
#define HERO_AGITATOR_CONSTANTS_HPP_

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
// Hero's waterwheel constants
static constexpr tap::algorithms::SmoothPidConfig WATERWHEEL_PID_CONFIG = {
    .kp = 8'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16000.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr float DESIRED_LOAD_TIME_S = .5f;
static constexpr float WATERWHEEL_NUM_BALL_POCKETS = 6.0f;
static constexpr float WATERWHEEL_TARGET_DISPLACEMENT = M_TWOPI / WATERWHEEL_NUM_BALL_POCKETS;
static constexpr float WATERWHEEL_TARGET_UNJAM_DISPLACEMENT =
    WATERWHEEL_TARGET_DISPLACEMENT / 10.0f;
static constexpr float WATERWHEEL_TARGET_UNJAM_TIME_S = 0.05f;

static constexpr aruwsrc::agitator::VelocityAgitatorSubsystemConfig WATERWHEEL_AGITATOR_CONFIG = {
    .gearRatio = 19.0f * 110 / 40,  // Number of teeth on pulleys * M3508
    .agitatorMotorId = tap::motor::MOTOR4,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = 0.75f * (WATERWHEEL_TARGET_DISPLACEMENT / DESIRED_LOAD_TIME_S),
    .jammingTime = 500,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 7000.0f,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config
    WATERWHEEL_AGITATOR_ROTATE_CONFIG = {
        .targetIntegralChange = WATERWHEEL_TARGET_DISPLACEMENT,
        .desiredSetpoint = WATERWHEEL_TARGET_DISPLACEMENT / DESIRED_LOAD_TIME_S,
        .integralSetpointTolerance = 0,  /// This tolerance can be 0 since the command considers
                                         /// itself done when the integral setpoint is >= initial
                                         /// integral + targetIntegralChange -
                                         /// integralSetpointTolerance. Thus, it is reasonable
                                         /// for this to be 0.
};

static constexpr tap::control::setpoint::UnjamIntegralCommand::Config
    WATERWHEEL_AGITATOR_UNJAM_CONFIG = {
        .targetUnjamIntegralChange = WATERWHEEL_TARGET_UNJAM_DISPLACEMENT,
        .unjamSetpoint = WATERWHEEL_TARGET_UNJAM_DISPLACEMENT / WATERWHEEL_TARGET_UNJAM_TIME_S,
        /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
        /// seconds. Add 500 ms extra tolerance.
        .maxWaitTime = static_cast<uint32_t>(1000.0f * WATERWHEEL_TARGET_UNJAM_TIME_S) + 500,
        .targetCycleCount = 1,
};

// PID terms for the hero kicker
static constexpr tap::algorithms::SmoothPidConfig KICKER_PID_CONFIG = {
    .kp = 5'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16000.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr aruwsrc::agitator::VelocityAgitatorSubsystemConfig KICKER_AGITATOR_CONFIG = {
    .gearRatio = 36.0f,
    .agitatorMotorId = tap::motor::MOTOR8,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,
    .jammingVelocityDifference = 0,
    .jammingTime = 0,
    .jamLogicEnabled = false,
    .velocityPIDFeedForwardGain = 0,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config
    KICKER_LOAD_AGITATOR_ROTATE_CONFIG = {
        .targetIntegralChange = M_PI / 2.0f,
        .desiredSetpoint = (M_PI / 2.0f) / DESIRED_LOAD_TIME_S,
        .integralSetpointTolerance = 0,  /// This tolerance can be 0 since the command considers
                                         /// itself done when the integral setpoint is >= initial
                                         /// integral + targetIntegralChange -
                                         /// integralSetpointTolerance. Thus, it is reasonable
                                         /// for this to be 0.
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config
    KICKER_SHOOT_AGITATOR_ROTATE_CONFIG = {
        .targetIntegralChange = M_PI,
        .desiredSetpoint = 6.0 * M_PI,
        .integralSetpointTolerance = 0,  /// This tolerance can be 0 since the command considers
                                         /// itself done when the integral setpoint is >= initial
                                         /// integral + targetIntegralChange -
                                         /// integralSetpointTolerance. Thus, it is reasonable
                                         /// for this to be 0.
};
static constexpr uint16_t HEAT_LIMIT_BUFFER = 100;
}  // namespace aruwsrc::control::agitator::constants

#endif  // HERO_AGITATOR_CONSTANTS_HPP_