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

#ifndef SOLDIER_AGITATOR_CONSTANTS_HPP_
#define SOLDIER_AGITATOR_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "../velocity_agitator_subsystem_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use agitator_constants.hpp instead.
#ifndef AGITATOR_CONSTANTS_HPP_
#error "Do not include this file directly! Use agitator_constants.hpp instead."
#endif

namespace aruwsrc::control::agitator::constants
{
// position PID terms
// PID terms for soldier
static constexpr tap::algorithms::SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 5'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16'000.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

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
    .targetIntegralChange = M_TWOPI / 10.0f,
    .desiredSetpoint = 2.0f * M_TWOPI,
    .integralSetpointTolerance = M_PI / 20.0f,
};

static constexpr tap::control::setpoint::UnjamIntegralCommand::Config AGITATOR_UNJAM_CONFIG = {
    .targetUnjamIntegralChange = M_TWOPI / 10.0f,
    .unjamSetpoint = M_TWOPI / 2.0f,
    /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
    /// seconds. Add 100 ms extra tolerance.
    .extraWaitTime = 100,
    .targetCycleCount = 3,
};

static constexpr uint16_t HEAT_LIMIT_BUFFER = 20;
}  // namespace aruwsrc::control::agitator::constants

#endif  // SOLDIER_AGITATOR_CONSTANTS_HPP_
