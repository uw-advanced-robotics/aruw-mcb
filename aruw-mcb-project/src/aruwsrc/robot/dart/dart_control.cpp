/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#if defined(TARGET_DART)

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"

#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/dart/dart_command.hpp"
#include "aruwsrc/robot/dart/dart_constants.hpp"
#include "aruwsrc/robot/dart/dart_drivers.hpp"
#include "aruwsrc/robot/dart/dart_subsystem.hpp"

using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::control;
using namespace tap::communication::serial;
using namespace aruwsrc::dart;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace dart_control
{
/* define subsystems ----------------------------------------------*/
tap::motor::DjiMotor pullMotor(drivers(), PULL_MOTOR_ID, CAN_BUS_MOTORS, false, "Pitch Turret");

DartSubsystem dart(drivers(), &pullMotor);

DartCommand dartCommand(dart, drivers());

HoldRepeatCommandMapping rightSwitchDown(
    drivers(),
    {&dartCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN),
    true);

aruwsrc::control::buzzer::BuzzerSubsystem buzzer(drivers());

/* only being used for the encoder motor */
tap::motor::DjiMotor deadMotor1(
    drivers(),
    DEAD_MOTOR1,
    CAN_BUS_MOTORS,
    false,
    "Pitch Turret Encoder Motor 1");
tap::motor::DjiMotor deadMotor2(
    drivers(),
    DEAD_MOTOR2,
    CAN_BUS_MOTORS,
    false,
    "Pitch Turret Encoder Motor 2");

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems()
{
    dart.initialize();
    buzzer.initialize();
}

void registerDartSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&dart);
    drivers->commandScheduler.registerSubsystem(&buzzer);
}

void setDefaultDartCommands(Drivers*)
{
    dart.setDefaultCommand(&dartCommand);
}

void startDartCommands(Drivers* drivers)
{
    drivers->commandScheduler.addCommand(&dartCommand);
    drivers->commandScheduler.addCommand(&lowBatteryCommand);
}

void registerDartIoMappings(Drivers* drivers) { drivers->commandMapper.addMap(&rightSwitchDown); }

}  // namespace dart_control

namespace aruwsrc::dart
{
void initSubsystemCommands(aruwsrc::dart::Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &dart_control::remoteSafeDisconnectFunction);
    dart_control::initializeSubsystems();
    dart_control::registerDartSubsystems(drivers);
    dart_control::setDefaultDartCommands(drivers);
    dart_control::startDartCommands(drivers);
    dart_control::registerDartIoMappings(drivers);
}
}  // namespace aruwsrc::dart

#endif
