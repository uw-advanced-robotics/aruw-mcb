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

#if defined(TARGET_HERO)

#include <aruwlib/DriversSingleton.hpp>
#include <aruwlib/control/CommandMapper.hpp>
#include <aruwlib/control/HoldCommandMapping.hpp>
#include <aruwlib/control/HoldRepeatCommandMapping.hpp>
#include <aruwlib/control/PressCommandMapping.hpp>
#include <aruwlib/control/ToggleCommandMapping.hpp>

#include "agitator/agitator_calibrate_command.hpp"
#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "aruwsrc/serial/xavier_serial.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "launcher/friction_wheel_rotate_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

using aruwlib::DoNotUse_getDrivers;
using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::launcher;
using namespace aruwsrc::turret;
using namespace aruwlib::control;
using aruwlib::DoNotUse_getDrivers;
using aruwlib::Remote;
using aruwlib::control::CommandMapper;
using aruwlib::control::RemoteMapState;
/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwlib::driversFunc drivers = aruwlib::DoNotUse_getDrivers;

/*
 * The xavier serial object is declared in main.cpp but is required by cv commands
 * and depends upon the chassis and turret subsystems, so declare it extern here.
 */
extern aruwsrc::serial::XavierSerial xavierSerial;

namespace aruwsrc
{
namespace control
{
/* define subsystems --------------------------------------------------------*/
TurretSubsystem turret(drivers());

ChassisSubsystem chassis(drivers());

// Hero has two agitators, one waterWheel and then a kicker
AgitatorSubsystem waterWheelAgitator(
    drivers(),
    AgitatorSubsystem::PID_HERO1_P,
    AgitatorSubsystem::PID_HERO1_I,
    AgitatorSubsystem::PID_HERO1_D,
    AgitatorSubsystem::PID_HERO1_MAX_ERR_SUM,
    AgitatorSubsystem::PID_HERO1_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::HERO1_AGITATOR_MOTOR_ID,
    AgitatorSubsystem::HERO1_AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem::HERO1_AGITATOR_INVERTED);

AgitatorSubsystem kickerAgitator(
    drivers(),
    AgitatorSubsystem::PID_HERO2_P,
    AgitatorSubsystem::PID_HERO2_I,
    AgitatorSubsystem::PID_HERO2_D,
    AgitatorSubsystem::PID_HERO2_MAX_ERR_SUM,
    AgitatorSubsystem::PID_HERO2_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::HERO2_AGITATOR_MOTOR_ID,
    AgitatorSubsystem::HERO2_AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem::HERO2_AGITATOR_INVERTED);

FrictionWheelSubsystem frictionWheels(drivers());

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(drivers(), &chassis, &turret);
WiggleDriveCommand wiggleDriveCommand(drivers(), &chassis, &turret);
TurretWorldRelativePositionCommand turretWorldRelativeCommand(drivers(), &turret, &chassis);

TurretCVCommand turretCVCommand(&xavierSerial, &turret);

AgitatorCalibrateCommand waterWheelAgitatorCalibrateCommand(&waterWheelAgitator);
ShootFastComprisedCommand waterWheelAgitatorShootFastCommand(drivers(), &waterWheelAgitator);
ShootSlowComprisedCommand waterWheelAgitatorShootSlowCommand(drivers(), &waterWheelAgitator);

AgitatorCalibrateCommand kickerAgitatorCalibrateCommand(&kickerAgitator);
ShootFastComprisedCommand kickerAgitatorShootFastCommand(drivers(), &kickerAgitator);
ShootSlowComprisedCommand kickerAgitatorShootSlowCommand(drivers(), &kickerAgitator);

FrictionWheelRotateCommand spinFrictionWheels(
    &frictionWheels,
    FrictionWheelRotateCommand::DEFAULT_WHEEL_RPM);
FrictionWheelRotateCommand stopFrictionWheels(&frictionWheels, 0);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&stopFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisDriveCommand, &turretCVCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&wiggleDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&waterWheelAgitatorShootFastCommand,
     &kickerAgitatorShootFastCommand},  // TODO: update these agitator commands
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
ToggleCommandMapping rToggled(drivers(), {&stopFrictionWheels}, RemoteMapState({Remote::Key::R}));

ToggleCommandMapping fToggled(drivers(), {&wiggleDriveCommand}, RemoteMapState({Remote::Key::F}));

HoldRepeatCommandMapping leftMousePressedShiftNotPressed(
    drivers(),
    {&waterWheelAgitatorShootFastCommand, &kickerAgitatorShootFastCommand},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::SHIFT}));

HoldCommandMapping leftMousePressedShiftPressed(
    drivers(),
    {&waterWheelAgitatorShootSlowCommand, &kickerAgitatorShootSlowCommand},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {Remote::Key::SHIFT}));

HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    chassis.initialize();
    waterWheelAgitator.initialize();
    kickerAgitator.initialize();
    frictionWheels.initialize();
    xavierSerial.attachChassis(&chassis);
    xavierSerial.attachTurret(&turret);
}

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&waterWheelAgitator);
    drivers->commandScheduler.registerSubsystem(&kickerAgitator);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands(aruwlib::Drivers *)
{
    chassis.setDefaultCommand(&chassisAutorotateCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(aruwlib::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&waterWheelAgitatorCalibrateCommand);
    drivers->commandScheduler.addCommand(&kickerAgitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(aruwlib::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rToggled);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedShiftNotPressed);
    drivers->commandMapper.addMap(&leftMousePressedShiftPressed);
    drivers->commandMapper.addMap(&rightMousePressed);
}

void initSubsystemCommands(aruwlib::Drivers *drivers)
{
    initializeSubsystems();
    registerHeroSubsystems(drivers);
    setDefaultHeroCommands(drivers);
    startHeroCommands(drivers);
    registerHeroIoMappings(drivers);
}

}  // namespace control

}  // namespace aruwsrc

#endif
