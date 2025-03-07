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

#ifndef MOTOR_TESTER_CONSTANTS_HPP_
#define MOTOR_TESTER_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem_config.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::motor_tester::constants
{
// position PID terms
// PID terms for standard
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 3'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16'000.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr int AGITATOR_NUM_POCKETS = 8;          // number of balls in one rotation
static constexpr float AGITATOR_MAX_ROF = 30.0f;        // balls per second
static constexpr float OVERSHOOT_FUDGE_FACTOR = 0.37f;  // how much agitator overshoots

static constexpr aruwsrc::agitator::VelocityAgitatorSubsystemConfig AGITATOR_CONFIG = {
    .gearRatio = 36.0f,
    .agitatorMotorId = tap::motor::MOTOR2,
    .agitatorCanBusId = tap::can::CanBus::CAN_BUS1,
    .isAgitatorInverted = false,
    /**
     * The jamming constants. Agitator is considered jammed if difference between the velocity
     * setpoint and actual velocity is > jammingVelocityDifference for > jammingTime.
     */
    .jammingVelocityDifference = 2.0f * M_TWOPI,
    .jammingTime = 200,  // Fudge factor because it's unjamming more than it should; used to be 100
    .jamLogicEnabled = true,
    .velocityPIDFeedForwardGain = 500.0f / M_TWOPI,
};

static constexpr tap::control::setpoint::MoveIntegralCommand::Config AGITATOR_ROTATE_CONFIG = {
    // magic numbers are fudge factors
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

tap::algorithms::SmoothPidConfig m2006VelocityPidConfig =
    {.kp = 50.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

tap::algorithms::SmoothPidConfig rm3508VelocityPidConfig =
    {.kp = 12.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

// untuned!!
tap::algorithms::SmoothPidConfig gm6020VelocityPidConfig =
    {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

}  // namespace aruwsrc::motor_tester::constants

#endif  // MOTOR_TESTER_CONSTANTS_HPP_
