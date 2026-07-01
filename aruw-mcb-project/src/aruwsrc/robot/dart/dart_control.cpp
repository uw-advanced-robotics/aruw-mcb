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
#include <memory>

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/double_dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "aruwsrc/communication/low_battery_buzzer_command.hpp"
#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/dart/dart_constants.hpp"
#include "aruwsrc/robot/dart/dart_drivers.hpp"
#include "aruwsrc/robot/dart/dart_launcher_subsystem.hpp"
#include "aruwsrc/robot/dart/dart_reloader_subsystem.hpp"

#include "dart_close_command.hpp"
#include "dart_open_command.hpp"
#include "dart_pullback_command.hpp"
#include "dart_release_command.hpp"
#include "rotate_magazine_command.hpp"

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
tap::motor::DoubleDjiMotor pullMotors(
    drivers(),
    UPPER_PULL_MOTOR_ID,
    LOWER_PULL_MOTOR_ID,
    LAUNCHER_CAN_BUS,
    LAUNCHER_CAN_BUS,
    true,
    true,
    "Upper Motor",
    "Lower Motor");

tap::motor::DjiMotor reloaderMotor(
    drivers(),
    RELOADER_MOTOR_ID,
    RELOADER_CAN_BUS,
    false,
    "Reloader Motor",
    false,
    1.0);

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

DartLauncherSubsystem dartLauncher(drivers(), pullMotors);

DartReloaderSubsystem reloader(drivers(), reloaderMotor);

DartReleaseCommand dartRelease(dartLauncher, MANUAL_RELEASE_DESIRED_OUTPUT);
DartPullbackCommand dartPullback(dartLauncher, MANUAL_PULLBACK_DESIRED_OUTPUT);

DartOpenCommand servoOpen(dartLauncher);
DartCloseCommand servoClose(dartLauncher);

RotateMagazineCommand rotateMagazine(reloader);

auto rightUpLeftUpRms = RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP);
auto rightUpLeftUp = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&dartPullback},
    &rightUpLeftUpRms);

auto rightUpLeftDownRms = RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP);
auto rightUpLeftDown = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&dartRelease},
    &rightUpLeftDownRms);

auto rightDownLeftUpRms = RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP);
auto rightDownLeftUp = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&servoOpen},
    &rightDownLeftUpRms);

auto rightDownLeftDownRms = RemoteMapState(Remote::SwitchState::UP, Remote::SwitchState::UP);
auto rightDownLeftDown = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command*>{&servoClose},
    &rightDownLeftDownRms);

auto rightMidLeftDownRms = RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::MID);
auto rightMidLeftDown = std::make_unique<PressCommandMapping>(
    drivers(),
    std::vector<Command*>{&rotateMagazine},
    &rightMidLeftDownRms);

void initializeSubsystems()
{
    dartLauncher.initialize();
    reloader.initialize();
}

void registerDartSubsystems(aruwsrc::dart::Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&dartLauncher);
    drivers->commandScheduler.registerSubsystem(&reloader);
    drivers->digital.configureInputPullMode(
        tap::gpio::Digital::B,
        tap::gpio::Digital::InputPullMode::PullUp);
}

void setDefaultDartCommands(aruwsrc::dart::Drivers*) {}

void startDartCommands(aruwsrc::dart::Drivers*) {}

void registerDartIoMappings(aruwsrc::dart::Drivers* drivers)
{
    // drivers->commandMapper.addMap(std::move(rightUpLeftUp));
    // drivers->commandMapper.addMap(std::move(rightUpLeftDown));
    // drivers->commandMapper.addMap(std::move(rightDownLeftUp));
    // drivers->commandMapper.addMap(std::move(rightDownLeftDown));
    drivers->commandMapper.addMap(std::move(rightMidLeftDown));
}

}  // namespace dart_control
namespace aruwsrc::dart
{
void initSubsystemCommands(Drivers* drivers)
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
