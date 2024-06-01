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

#ifndef SENTRY_AGITATOR_CONSTANTS_HPP_
#define SENTRY_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/velocity_agitator_subsystem_config.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

// @todo update agitator constants for new agitator
namespace aruwsrc::control::agitator::constants
{
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 3'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 5'000.0f,
    .maxOutput = 16'000.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr int AGITATOR_NUM_POCKETS = 8;    // number of balls in one rotation
static constexpr float AGITATOR_MAX_ROF = 30.0f;  // balls per second

namespace turretLeft
{
static constexpr aruwsrc::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .gearRatio = 36.0f,
    .agitatorMotorId = tap::motor::MOTOR4,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS2,
    .isAgitatorInverted = false,  // @todo: check
    /**
     * The jamming constants. Agitator is considered jammed if difference between setpoint
     * and current angle is > `JAMMING_DISTANCE` radians for >= `JAMMING_TIME` ms;
     *
     * @warning: `JAMMING_DISTANCE` must be less than the smallest movement command
     *
     * This should be positive or else weird behavior can occur
     */
    .jammingVelocityDifference = M_TWOPI,
    .jammingTime = 300,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 500.0f / M_TWOPI,
};
}  // namespace turretLeft

namespace turretRight
{
static constexpr aruwsrc::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .gearRatio = 36.0f,
    .agitatorMotorId = tap::motor::MOTOR4,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,  // @todo: check
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = M_TWOPI,
    .jammingTime = 300,
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 500.0f / M_TWOPI,
};
}  // namespace turretRight

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG = {
    // magic numbers are fudge factors
    .targetIntegralChange = 1.2f * (M_TWOPI / AGITATOR_NUM_POCKETS),
    .desiredSetpoint = AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
    .integralSetpointTolerance = (M_TWOPI / AGITATOR_NUM_POCKETS) * 0.25f,
};

static constexpr tap::control::setpoint::UnjamIntegralCommand::Config AGITATOR_UNJAM_CONFIG = {
    // magic numbers are fudge factors
    .targetUnjamIntegralChange = 0.4f * (M_TWOPI / AGITATOR_NUM_POCKETS),
    .unjamSetpoint = 0.07f * AGITATOR_MAX_ROF * (M_TWOPI / AGITATOR_NUM_POCKETS),
    /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
    /// seconds.Convert to ms, Add 100 ms extra tolerance.
    .maxWaitTime = static_cast<uint32_t>(
                       1000.0f * (M_TWOPI / AGITATOR_NUM_POCKETS) / 0.2f * AGITATOR_MAX_ROF *
                       (M_TWOPI / AGITATOR_NUM_POCKETS)) +
                   100,
    .targetCycleCount = 3,
};

static constexpr uint16_t HEAT_LIMIT_BUFFER = 20;

/// Time in milliseconds to pause launching projectiles when the user requests the projectile
/// launcher to be paused
static constexpr uint32_t AGITATOR_PAUSE_PROJECTILE_LAUNCHING_TIME = 8'000;

}  // namespace aruwsrc::control::agitator::constants

#endif  // SENTRY_AGITATOR_CONSTANTS_HPP_
