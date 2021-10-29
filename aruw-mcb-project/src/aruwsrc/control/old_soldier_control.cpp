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

#if defined(TARGET_OLD_SOLDIER)

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers_singleton.hpp"

#include "agitator/agitator_shoot_comprised_command_instances.hpp"
#include "agitator/agitator_subsystem.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "hopper-cover/hopper_subsystem.hpp"
#include "hopper-cover/open_hopper_command_old.hpp"
#include "turret/turret_cv_command.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/turret_world_relative_position_command.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::control;
using tap::DoNotUse_getDrivers;
using tap::Remote;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
tap::driversFunc drivers = tap::DoNotUse_getDrivers;

namespace old_soldier_control
{
/* define subsystems --------------------------------------------------------*/
TurretSubsystem turret(drivers());

ChassisSubsystem chassis(drivers());

AgitatorSubsystem agitator(
    drivers(),
    AgitatorSubsystem::PID_17MM_P,
    AgitatorSubsystem::PID_17MM_I,
    AgitatorSubsystem::PID_17MM_D,
    AgitatorSubsystem::PID_17MM_MAX_ERR_SUM,
    AgitatorSubsystem::PID_17MM_MAX_OUT,
    AgitatorSubsystem::AGITATOR_GEAR_RATIO_M2006,
    AgitatorSubsystem::AGITATOR_MOTOR_ID,
    AgitatorSubsystem::AGITATOR_MOTOR_CAN_BUS,
    AgitatorSubsystem::isAgitatorInverted);

HopperSubsystem hopperCover(
    drivers(),
    tap::gpio::Pwm::W,
    HopperSubsystem::OLD_SOLDIER_HOPPER_OPEN_PWM,
    HopperSubsystem::OLD_SOLDIER_HOPPER_CLOSE_PWM,
    HopperSubsystem::OLD_SOLDIER_PWM_RAMP_SPEED);

/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

ChassisAutorotateCommand chassisAutorotateCommand(drivers(), &chassis, &turret);

WiggleDriveCommand wiggleDriveCommand(drivers(), &chassis, &turret);

TurretWorldRelativePositionCommand turretWorldRelativeCommand(drivers(), &turret, &chassis);

CalibrateCommand agitatorCalibrateCommand(&agitator);

MoveUnjamRefLimitedCommand agitatorShootFastCommand(
    drivers(),
    &agitator,
    M_PI / 5.0f,
    M_PI / 2.0f,
    10,
    false,
    0);

OpenHopperCommand openHopperCommand(&hopperCover);

/* define command mappings --------------------------------------------------*/
// Remote related mappings
HoldCommandMapping leftSwitchDown(
    drivers(),
    {&chassisDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&wiggleDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&agitatorShootFastCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Keyboard/Mouse related mappings
ToggleCommandMapping fToggled(drivers(), {&wiggleDriveCommand}, RemoteMapState({Remote::Key::F}));
HoldRepeatCommandMapping leftMousePressedShiftNotPressed(
    drivers(),
    {&agitatorShootFastCommand},
    RemoteMapState(RemoteMapState::MouseButton::LEFT, {}, {Remote::Key::SHIFT}));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    turret.initialize();
    chassis.initialize();
    agitator.initialize();
    hopperCover.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerOldSoldierSubsystems(tap::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&hopperCover);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultOldSoldierCommands(tap::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretWorldRelativeCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startOldSoldierCommands(tap::Drivers *drivers)
{
    drivers->commandScheduler.addCommand(&agitatorCalibrateCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerOldSoldierIoMappings(tap::Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&fToggled);
    drivers->commandMapper.addMap(&leftMousePressedShiftNotPressed);
}
}  // namespace old_soldier_control

namespace aruwsrc::control
{
void initSubsystemCommands(tap::Drivers *drivers)
{
    old_soldier_control::initializeSubsystems();
    old_soldier_control::registerOldSoldierSubsystems(drivers);
    old_soldier_control::setDefaultOldSoldierCommands(drivers);
    old_soldier_control::startOldSoldierCommands(drivers);
    old_soldier_control::registerOldSoldierIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif
