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
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/motortester/constant_rpm_command.hpp"
#include "aruwsrc/robot/motortester/motor_subsystem.hpp"
#include "aruwsrc/robot/motortester/motortester_drivers.hpp"
#include "aruwsrc/robot/motortester/stick_rpm_command.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace aruwsrc::motortester;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace motortester_control
{

tap::algorithms::SmoothPidConfig m2006PidConfig =
    {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

tap::algorithms::SmoothPidConfig m3508PidConfig =
    {.kp = 12.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 0.0f, .maxOutput = 16000.0f};

// unfinished 2006
tap::motor::DjiMotor leftChannelMotor(
    drivers(),
    tap::motor::MOTOR2,          // id 2
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "LMotor");

// 3508
tap::motor::DjiMotor rightChannelMotor(
    drivers(),
    tap::motor::MOTOR1,          // id 1
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "RMotor");

MotorSubsystem leftMotorSubsystem(drivers(), leftChannelMotor, m2006PidConfig, 1.0f);

MotorSubsystem rightMotorSubsystem(
    drivers(),
    rightChannelMotor,
    m3508PidConfig,
    (187.0f / 3591.0f));  // internal gearbox ratio

// Commands

ConstantRpmCommand leftSwitchUpRpm(&leftMotorSubsystem, 225.0f);
StickRpmCommand leftManual(
    &leftMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_VERTICAL,
    500.0f);
ConstantRpmCommand leftSwitchDownRpm(&leftMotorSubsystem, 187.5f);

ConstantRpmCommand rightSwitchUpRpm(&rightMotorSubsystem, 30.0f, 32.0f / 110.0f);
StickRpmCommand rightManual(
    &rightMotorSubsystem,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::RIGHT_VERTICAL,
    482.0f);
ConstantRpmCommand rightSwitchDownRpm(&rightMotorSubsystem, 15.0f, 32.0f / 152.0f);

// command mappings

tap::control::HoldCommandMapping leftSwitchUp(
    drivers(),
    {&leftSwitchUpRpm},
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
    // leftMotorSubsystem.initialize();
    rightMotorSubsystem.initialize();
}

void registerSubsystems(Drivers *drivers)
{
    // drivers->commandScheduler.registerSubsystem(&leftMotorSubsystem);
    drivers->commandScheduler.registerSubsystem(&rightMotorSubsystem);
}

void setDefaultCommands(Drivers *drivers) {}

void registerIoMappings(Drivers *drivers)
{
    // drivers->commandMapper.addMap(&leftSwitchUp);
    // drivers->commandMapper.addMap(&leftSwitchDown);
    // drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchMid);
}

}  // namespace motortester_control

namespace aruwsrc::motortester
{
void initSubsystemCommands(aruwsrc::motortester::Drivers *drivers)
{
    motortester_control::registerSubsystems(drivers);
    motortester_control::initializeSubsystems();
    motortester_control::setDefaultCommands(drivers);
    motortester_control::registerIoMappings(drivers);
}

}  // namespace aruwsrc::motortester

#endif
