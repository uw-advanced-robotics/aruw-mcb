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

#if defined(TARGET_CHARACTERIZER)

#include "tap/control/command.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/motor/tmotor_ak80_9.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/characterizer/characterizer_drivers.hpp"
#include "aruwsrc/robot/characterizer/output_sweep_command.hpp"
#include "aruwsrc/robot/characterizer/raw_motor_subsystem.hpp"
#include "aruwsrc/robot/characterizer/stick_output_command.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace aruwsrc::characterizer;
using namespace tap::communication::serial;
using namespace tap::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace characterizer_control
{
aruwsrc::control::motor::Tmotor_AK809 motor(
    drivers(),
    aruwsrc::control::motor::TMotorId::MOTOR4,
    tap::can::CanBus::CAN_BUS1,
    false,
    "LMotor");

RawMotorSubsystem motorSubsystem(drivers(), motor, true);

// ----------
// Commands
// ----------

StickOutputCommand manual(
    &motorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_VERTICAL,
    60000.0f);

StickOutputCommand manualFine(
    &motorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_VERTICAL,
    1000.0f);

OutputSweepCommand sweep(
    &motorSubsystem,
    drivers()->digital,
    tap::gpio::Digital::OutputPin::E,
    0,      // min output
    16500,  // max output
    500,    // step length (ms)
    250,    // step size
    -1);

Command* sweepHeld(sweep.andThen(motorSubsystem.run([]() {})));
Trigger leftSwitchUp =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)
        .whileTrue(sweepHeld);

Trigger leftSwitchDown =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN)
        .whileTrue(&manual);

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// inits
void initializeSubsystems() { motorSubsystem.initialize(); }

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &characterizer_control::remoteSafeDisconnectFunction);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem);
}

void registerIoMappings(Drivers*) { motorSubsystem.setDefaultCommand(&manualFine); }

}  // namespace characterizer_control

namespace aruwsrc::characterizer
{
void initSubsystemCommands(aruwsrc::characterizer::Drivers* drivers)
{
    characterizer_control::registerSubsystems(drivers);
    characterizer_control::initializeSubsystems();
    characterizer_control::registerIoMappings(drivers);
}

}  // namespace aruwsrc::characterizer

#endif
