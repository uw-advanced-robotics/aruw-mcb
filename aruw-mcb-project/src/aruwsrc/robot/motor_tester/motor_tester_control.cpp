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

#if defined(TARGET_MOTOR_TESTER)

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/motor_tester/constant_rpm_command.hpp"
#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_constants.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_drivers.hpp"
#include "aruwsrc/robot/motor_tester/stick_rpm_command.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace aruwsrc::motor_tester;
using namespace aruwsrc::motor_tester::constants;
using namespace aruwsrc::agitator;
using namespace aruwsrc::control::agitator;
using namespace tap::control::setpoint;
// using namespace tap::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace motor_tester_control
{
// m2006
tap::motor::DjiMotor leftChannelMotor(
    drivers(),
    tap::motor::MOTOR3,          // id 3
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "LMotor");

VelocityAgitatorSubsystem agitator(drivers(), AGITATOR_PID_CONFIG, AGITATOR_CONFIG);

// 3508
tap::motor::DjiMotor rightChannelMotor(
    drivers(),
    tap::motor::MOTOR1,          // id 1
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "RMotor");

// 6020
tap::motor::DjiMotor wheelChannelMotor(
    drivers(),
    tap::motor::MOTOR7,          // id 3+4
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "WMotor");

MotorSubsystem leftMotorSubsystem(
    drivers(),
    leftChannelMotor,
    m2006VelocityPidConfig,
    (1.0f / 36.0f));

MotorSubsystem rightMotorSubsystem(
    drivers(),
    rightChannelMotor,
    rm3508VelocityPidConfig,
    (187.0f / 3591.0f));  // internal gearbox ratio

MotorSubsystem wheelMotorSubsystem(
    drivers(),
    wheelChannelMotor,
    gm6020VelocityPidConfig,
    (1.0f));  // internal gearbox ratio

// ----------
// Commands
// ----------

StickRpmCommand leftManual(
    &leftMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_VERTICAL,
    500.0f);

StickRpmCommand rightManual(
    &rightMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::RIGHT_VERTICAL,
    482.0f);

StickRpmCommand wheelManual(
    &wheelMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::WHEEL,
    320.0f);

// agitator rotate/unjam commands
MoveIntegralCommand rotateAgitator(agitator, AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// ------------------
// command mappings
// ------------------

tap::control::HoldRepeatCommandMapping leftSwitchUp(
    drivers(),
    {&rotateAndUnjamAgitator},
    tap::control::RemoteMapState(
        tap::communication::serial::Remote::Switch::LEFT_SWITCH,
        tap::communication::serial::Remote::SwitchState::UP),
    true);

// inits

void initializeSubsystems()
{
    agitator.initialize();
    leftMotorSubsystem.initialize();
    rightMotorSubsystem.initialize();
    wheelMotorSubsystem.initialize();
}

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&leftMotorSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&rightMotorSubsystem);
    drivers->commandScheduler.registerSubsystem(&wheelMotorSubsystem);
}

void registerIoMappings(Drivers* drivers)
{
    drivers->commandMapper.addMap(&leftSwitchUp);

    wheelMotorSubsystem.setDefaultCommand(&wheelManual);
    leftMotorSubsystem.setDefaultCommand(&leftManual);
    rightMotorSubsystem.setDefaultCommand(&rightManual);
}

}  // namespace motor_tester_control

namespace aruwsrc::motor_tester
{
void initSubsystemCommands(aruwsrc::motor_tester::Drivers* drivers)
{
    motor_tester_control::registerSubsystems(drivers);
    motor_tester_control::initializeSubsystems();
    motor_tester_control::registerIoMappings(drivers);
}

}  // namespace aruwsrc::motor_tester

#endif
