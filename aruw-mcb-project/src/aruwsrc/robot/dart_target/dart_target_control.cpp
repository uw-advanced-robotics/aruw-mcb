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

#if defined(TARGET_LAUNCHER_TARGET)

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/dart_target/dart_target_constants.hpp"
#include "aruwsrc/robot/dart_target/dart_target_drivers.hpp"
#include "aruwsrc/robot/dart_target/motor_subsystem.hpp"
#include "aruwsrc/robot/dart_target/stick_rpm_command.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace tap::control::setpoint;

using namespace aruwsrc::control::agitator;
using namespace aruwsrc::dart_target;
using namespace aruwsrc::dart_target::constants;
// using namespace tap::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace dart_target_control
{
// m2006
tap::motor::DjiMotor motor2006(
    drivers(),
    tap::motor::MOTOR3,          // id 3
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "2006 Motor",
    true,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006);

MotorSubsystem motorSubsystem2006(drivers(), motor2006, m2006VelocityPidConfig);

// ----------
// Commands
// ----------

StickRpmCommand leftVerticalManual(
    &motorSubsystem2006,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_VERTICAL,
    500.0f);

// ------------------
// command mappings
// ------------------

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// inits

void initializeSubsystems() { motorSubsystem2006.initialize(); modm::platform::RandomNumberGenerator::enable();}

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &dart_target_control::remoteSafeDisconnectFunction);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem2006);
}

void registerIoMappings(Drivers*) { motorSubsystem2006.setDefaultCommand(&leftVerticalManual); }

}  // namespace dart_target_control

namespace aruwsrc::dart_target
{
void initSubsystemCommands(aruwsrc::dart_target::Drivers* drivers)
{
    dart_target_control::registerSubsystems(drivers);
    dart_target_control::initializeSubsystems();
    dart_target_control::registerIoMappings(drivers);
}

}  // namespace aruwsrc::dart_target

#endif
