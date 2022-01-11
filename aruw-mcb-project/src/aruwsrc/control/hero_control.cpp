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

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "aruwsrc/drivers_singleton.hpp"
#include "chassis/chassis_autorotate_command.hpp"
#include "chassis/chassis_drive_command.hpp"
#include "chassis/chassis_subsystem.hpp"
#include "chassis/wiggle_drive_command.hpp"
#include "client-display/client_display_command.hpp"
#include "client-display/client_display_subsystem.hpp"
#include "launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "launcher/friction_wheel_subsystem.hpp"
#include "turret/turret_subsystem.hpp"
#include "turret/world-relative/turret_world_relative_command.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control::turret;
using namespace tap::control;
using namespace aruwsrc::display;
using namespace aruwsrc::control::launcher;
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

tap::motor::DjiMotor pitchMotor(
    drivers(),
    TurretSubsystem::PITCH_MOTOR_ID,
    TurretSubsystem::CAN_BUS_MOTORS,
    true,
    "Pitch Turret");
tap::motor::DoubleDjiMotor yawMotor(
    drivers(),
    TurretSubsystem::YAW_BACK_MOTOR_ID,
    TurretSubsystem::YAW_FRONT_MOTOR_ID,
    TurretSubsystem::CAN_BUS_MOTORS,
    TurretSubsystem::CAN_BUS_MOTORS,
    true,
    true,
    "Yaw Back Turret",
    "Yaw Front Turret");
TurretSubsystem turret(drivers(), &pitchMotor, &yawMotor, false);


/* define commands ----------------------------------------------------------*/
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    10.0f,
    false,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_42MM);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_42MM);

ClientDisplayCommand clientDisplayCommand(
    drivers(),
    &clientDisplay,
    nullptr,
    nullptr,
    nullptr,
    &chassisDriveCommand);

TurretChassisRelativeCommand turretChassisRelativeCommand(drivers(), &turret);

TurretWorldRelativeCommand turretWorldRelativeCommand(drivers(), &turret);

TurretCVCommand turretCVCommand(drivers(), &turret);

TurretQuickTurnCommand turretUTurnCommand(&turret, 180.0f);

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

// Keyboard/Mouse related mappings
HoldCommandMapping rightMousePressed(
    drivers(),
    {&turretCVCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

PressCommandMapping zPressed(drivers(), {&turretUTurnCommand}, RemoteMapState({Remote::Key::Z}));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    frictionWheels.initialize();
    clientDisplay.initialize();
    turret.initialize();
    drivers()->legacyVisionCoprocessor.attachChassis(&chassis);
}

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandSchedule.registerSubsystem(&turret);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands(aruwsrc::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    frictionWheels.setDefaultCommand(&spinFrictionWheels);
    clientDisplay.setDefaultCommand(&clientDisplayCommand);
    turret.setDefaultCommand(&turretChassisRelativeCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(aruwsrc::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rightMousePressed);
    drivers->commandMapper.addMap(&zPressed);
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

#endif
