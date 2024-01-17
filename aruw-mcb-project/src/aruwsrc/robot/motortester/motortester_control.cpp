/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
// #include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/motortester/constant_rpm_command.hpp"
#include "aruwsrc/robot/motortester/motor_subsystem.hpp"
#include "aruwsrc/robot/motortester/motortester_constants.hpp"
#include "aruwsrc/robot/motortester/motortester_drivers.hpp"
#include "aruwsrc/robot/motortester/stick_rpm_command.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace aruwsrc::motortester;
using namespace aruwsrc::motortester::constants;
using namespace aruwsrc::agitator;
using namespace tap::control::setpoint;
// using namespace tap::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace motortester_control
{

// // m2006
tap::motor::DjiMotor leftChannelMotor(
    drivers(),
    tap::motor::MOTOR2,          // id 2
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
    tap::motor::MOTOR7,          // id 3
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

ConstantRpmCommand leftSwitchUpRpm(&leftMotorSubsystem, 225.0f);
StickRpmCommand leftManual(
    &leftMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_VERTICAL,
    500.0f);
ConstantRpmCommand leftSwitchDownRpm(&leftMotorSubsystem, 187.5f);

ConstantRpmCommand rightSwitchUpRpm(&rightMotorSubsystem, 180.0f);  // 32.0f / 110.0f
StickRpmCommand rightManual(
    &rightMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::RIGHT_VERTICAL,
    482.0f);
ConstantRpmCommand rightSwitchDownRpm(&rightMotorSubsystem, 150.0f);  // 32.0f / 152.0f

StickRpmCommand wheelManual(
    &wheelMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::WHEEL,
    320.0f);

// base rotate/unjam commands
MoveIntegralCommand rotateAgitator(agitator, AGITATOR_ROTATE_CONFIG);

UnjamIntegralCommand unjamAgitator(agitator, AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// ------------------
// command mappings
// ------------------

tap::control::HoldCommandMapping leftSwitchUp(
    drivers(),
    {&rotateAndUnjamAgitator},
    tap::control::RemoteMapState(
        tap::communication::serial::Remote::Switch::LEFT_SWITCH,
        tap::communication::serial::Remote::SwitchState::UP));

tap::control::HoldCommandMapping leftSwitchMid(
    drivers(),
    {&leftManual},
    tap::control::RemoteMapState(
        tap::communication::serial::Remote::Switch::LEFT_SWITCH,
        tap::communication::serial::Remote::SwitchState::MID));

tap::control::HoldCommandMapping leftSwitchDown(
    drivers(),
    {&leftSwitchDownRpm},
    tap::control::RemoteMapState(
        tap::communication::serial::Remote::Switch::LEFT_SWITCH,
        tap::communication::serial::Remote::SwitchState::DOWN));

tap::control::HoldCommandMapping rightSwitchUp(
    drivers(),
    {&rightSwitchUpRpm},
    tap::control::RemoteMapState(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::UP));

tap::control::HoldCommandMapping rightSwitchMid(
    drivers(),
    {&rightManual},
    tap::control::RemoteMapState(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::MID));

tap::control::HoldCommandMapping rightSwitchDown(
    drivers(),
    {&rightSwitchDownRpm},
    tap::control::RemoteMapState(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::DOWN));

// inits

void initializeSubsystems()
{
    agitator.initialize();
    // leftMotorSubsystem.initialize();
    rightMotorSubsystem.initialize();
    wheelMotorSubsystem.initialize();
}

void registerSubsystems(Drivers* drivers)
{
    // drivers->commandScheduler.registerSubsystem(&leftMotorSubsystem);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&rightMotorSubsystem);
    drivers->commandScheduler.registerSubsystem(&wheelMotorSubsystem);
}

void registerIoMappings(Drivers* drivers)
{
    drivers->commandMapper.addMap(&leftSwitchUp);
    // drivers->commandMapper.addMap(&leftSwitchDown);
    // drivers->commandMapper.addMap(&leftSwitchMid);

    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchMid);

    wheelMotorSubsystem.setDefaultCommand(&wheelManual);
}

}  // namespace motortester_control

namespace aruwsrc::motortester
{
void initSubsystemCommands(aruwsrc::motortester::Drivers* drivers)
{
    motortester_control::registerSubsystems(drivers);
    motortester_control::initializeSubsystems();
    motortester_control::registerIoMappings(drivers);
}

}  // namespace aruwsrc::motortester

#endif
