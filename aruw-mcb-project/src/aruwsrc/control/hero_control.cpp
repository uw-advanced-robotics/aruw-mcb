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

//#if defined(TARGET_HERO)

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/agitator/hero_agitator_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/world-relative/turret_world_relative_command.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::display;
using namespace aruwsrc::agitator;
using tap::Remote;
using tap::control::CommandMapper;
using tap::control::RemoteMapState;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace hero_control
{
/* define subsystems --------------------------------------------------------*/
ChassisSubsystem chassis(drivers());

FrictionWheelSubsystem frictionWheels(drivers());

ClientDisplaySubsystem clientDisplay(drivers());

AgitatorSubsystem kickerAgitator(
    drivers(),
    AgitatorSubsystem::PID_HERO_KICKER_P,
    AgitatorSubsystem::PID_HERO_KICKER_I,
    AgitatorSubsystem::PID_HERO_KICKER_D,
    AgitatorSubsystem::PID_HERO_KICKER_MAX_ERR_SUM,
    AgitatorSubsystem::PID_HERO_KICKER_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::HERO_KICKER_MOTOR_ID,
    AgitatorSubsystem::HERO_KICKER_MOTOR_CAN_BUS,
    AgitatorSubsystem::HERO_KICKER_INVERTED,
    false,
    0,
    0);

AgitatorSubsystem waterwheelAgitator(
    drivers(),
    AgitatorSubsystem::PID_HERO_WATERWHEEL_P,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_I,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_D,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_MAX_ERR_SUM,
    AgitatorSubsystem::PID_HERO_WATERWHEEL_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_GM3508,
    AgitatorSubsystem::HERO_WATERWHEEL_MOTOR_ID,
    AgitatorSubsystem::HERO_WATERWHEEL_MOTOR_CAN_BUS,
    AgitatorSubsystem::HERO_WATERWHEEL_INVERTED,
    true,
    AgitatorSubsystem::JAM_DISTANCE_TOLERANCE_WATERWHEEL,
    AgitatorSubsystem::JAM_TEMPORAL_TOLERANCE_WATERWHEEL);

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

FrictionWheelRotateCommand spinFrictionWheels(
    &frictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);
FrictionWheelRotateCommand stopFrictionWheels(&frictionWheels, 0);

ClientDisplayCommand clientDisplayCommand(
    drivers(),
    &clientDisplay,
    nullptr,
    nullptr,
    nullptr,
    &chassisDriveCommand);

HeroAgitatorCommand heroAgitatorCommand(
    drivers(),
    &kickerAgitator,
    &waterwheelAgitator,
    M_PI / 2.0,
    75,
    M_PI / 2.0,
    M_PI / 10.0,
    100,
    M_PI / 2.0,
    true,
    100);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&heroAgitatorCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
PressCommandMapping leftMousePressed(
    drivers(),
    {&heroAgitatorCommand},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    frictionWheels.initialize();
    clientDisplay.initialize();
    kickerAgitator.initialize();
    waterwheelAgitator.initialize();
    drivers()->xavierSerial.attachChassis(&chassis);
}

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&kickerAgitator);
    drivers->commandScheduler.registerSubsystem(&waterwheelAgitator);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands(aruwsrc::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(aruwsrc::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&leftMousePressed);
}
}  // namespace hero_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    hero_control::initializeSubsystems();
    hero_control::registerHeroSubsystems(drivers);
    hero_control::setDefaultHeroCommands(drivers);
    hero_control::startHeroCommands(drivers);
    hero_control::registerHeroIoMappings(drivers);
}
}  // namespace aruwsrc::control

//#endif
