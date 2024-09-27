/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef STANDARD_AGITATOR_CONSTANTS_HPP_
#define STANDARD_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

using tap::motor::DjiMotor;

namespace aruwsrc::control::agitator::constants
{
/// How much extra heat must be available beyond how much it takes to fire the next shot
static constexpr uint16_t HEAT_LIMIT_BUFFER = 25;

#if defined(TARGET_STANDARD_ORION) || defined(TARGET_STANDARD_CYGNUS)

// position PID terms
// PID terms for standard
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 5'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_C610,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr int AGITATOR_NUM_POCKETS = 8;          // number of balls in one rotation
static constexpr float AGITATOR_MAX_ROF = 30.0f;        // balls per second
static constexpr float OVERSHOOT_FUDGE_FACTOR = 0.37f;  // how much agitator overshoots

static constexpr aruwsrc::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .gearRatio = 36.0f,
    .agitatorMotorId = tap::motor::MOTOR7,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = M_TWOPI,
    .jammingTime = 100,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 500.0f / M_TWOPI,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG = {
    .targetIntegralChange = M_TWOPI / AGITATOR_NUM_POCKETS - OVERSHOOT_FUDGE_FACTOR,
    .desiredSetpoint = AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
    .integralSetpointTolerance = (M_TWOPI / AGITATOR_NUM_POCKETS) * 0.1f,
};

constexpr float UNJAM_VELOCITY = 0.35 * AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS);
constexpr float UNJAM_DISTANCE = 0.6f * (M_TWOPI / AGITATOR_NUM_POCKETS);
static constexpr aruwsrc::control::agitator::UnjamSpokeAgitatorCommand::Config
    AGITATOR_UNJAM_CONFIG = {
        .targetUnjamIntegralChange = UNJAM_DISTANCE,
        .unjamSetpoint = UNJAM_VELOCITY,
        /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
        /// seconds.Convert to ms, Add 100 ms extra tolerance.
        .maxWaitTime = static_cast<uint32_t>(1000.0f * UNJAM_DISTANCE / UNJAM_VELOCITY) + 200,
        .targetCycleCount = 3,
};

#elif defined(TARGET_STANDARD_SPIDER) || defined(TARGET_STANDARD_ELSA)

// position PID terms
// PID terms for standard
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 5'000.0f,
    .ki = 0.0001f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_C610,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr int AGITATOR_NUM_POCKETS = 10;   // number of balls in one rotation
static constexpr float AGITATOR_MAX_ROF = 20.0f;  // balls per second

static constexpr aruwsrc::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .gearRatio = 36.0f,
    .agitatorMotorId = tap::motor::MOTOR7,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = M_TWOPI,
    .jammingTime = 100,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 500.0f / M_TWOPI,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG = {
    .targetIntegralChange =
        1.1f * (M_TWOPI / AGITATOR_NUM_POCKETS),  // @todo remove multiplier if possible
    .desiredSetpoint = AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
    .integralSetpointTolerance = (M_TWOPI / AGITATOR_NUM_POCKETS) * 0.25f,
};

static constexpr aruwsrc::control::agitator::UnjamSpokeAgitatorCommand::Config
    AGITATOR_UNJAM_CONFIG = {
        .targetUnjamIntegralChange = (M_TWOPI / AGITATOR_NUM_POCKETS),
        .unjamSetpoint = 0.25f * AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
        /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
        /// seconds.Convert to ms, Add 100 ms extra tolerance.
        .maxWaitTime = static_cast<uint32_t>(
                           1000.0f * (M_TWOPI / AGITATOR_NUM_POCKETS) / 0.25f * AGITATOR_MAX_ROF *
                           (M_TWOPI / AGITATOR_NUM_POCKETS)) +
                       100,
        .targetCycleCount = 3,
};

#else
#error "Attempted to include standard_agitator_constants.hpp for nonstandard robot target."
#endif

}  // namespace aruwsrc::control::agitator::constants

#endif  // STANDARD_AGITATOR_CONSTANTS_HPP_
