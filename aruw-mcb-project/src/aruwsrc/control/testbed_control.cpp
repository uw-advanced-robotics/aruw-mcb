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

#include "aruwsrc/util_macros.hpp"

#ifdef TARGET_TESTBED

#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/display/imu_calibrate_menu.hpp"
#include "aruwsrc/drivers_singleton.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace tap::control;
using namespace aruwsrc::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace constants
{
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
    .targetIntegralChange = 1.1f * (M_TWOPI / 10.0f),
    .desiredSetpoint = 2.0f * M_TWOPI,
    .integralSetpointTolerance = M_PI / 20.0f,
};

static constexpr tap::control::setpoint::UnjamIntegralCommand::Config AGITATOR_UNJAM_CONFIG = {
    .targetUnjamIntegralChange = M_TWOPI / 10.0f,
    .unjamSetpoint = M_TWOPI / 2.0f,
    /// Unjamming should take unjamDisplacement (radians) / unjamVelocity (radians / second)
    /// seconds. Add 100 ms extra tolerance.
    .maxWaitTime = static_cast<uint32_t>(1000.0f * (M_TWOPI / 15.0f) / (M_TWOPI / 4.0f)) + 100,
    .targetCycleCount = 3,
};
}  // namespace constants

namespace testbed_control
{
/* define subsystems --------------------------------------------------------*/

VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

/* define commands-----------------------------------------------------------*/
// base rotate/unjam commands
MoveIntegralCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
// HoldCommandMapping rightSwitchDown(
//     drivers(),
//     {&stopFrictionWheels},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&rotateAndUnjamAgitator},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerTestbedSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() { agitator.initialize(); }

/* set any default commands to subsystems here ------------------------------*/
void setDefaultTestbedCommands(aruwsrc::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startTestbedCommands(aruwsrc::Drivers *drivers) {}

/* register io mappings here ------------------------------------------------*/
void registerTestbedIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchUp);
}
}  // namespace testbed_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &testbed_control::remoteSafeDisconnectFunction);
    testbed_control::initializeSubsystems();
    testbed_control::registerTestbedSubsystems(drivers);
    testbed_control::setDefaultTestbedCommands(drivers);
    testbed_control::startTestbedCommands(drivers);
    testbed_control::registerTestbedIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif
