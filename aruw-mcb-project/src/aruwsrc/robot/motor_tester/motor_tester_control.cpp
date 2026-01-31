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
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/motor_tester/constant_rpm_command.hpp"
#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_constants.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_drivers.hpp"
#include "aruwsrc/robot/motor_tester/stick_rpm_command.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace tap::control::setpoint;

using namespace aruwsrc::control::agitator;
using namespace aruwsrc::motor_tester;
using namespace aruwsrc::motor_tester::constants;
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
tap::motor::DjiMotor motor2006(
    drivers(),
    tap::motor::MOTOR3,          // id 3
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "2006 Motor",
    true,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006);

VelocityAgitatorSubsystem agitator(drivers(), AGITATOR_PID_CONFIG, AGITATOR_CONFIG);

// 3508
tap::motor::DjiMotor motor3508(
    drivers(),
    tap::motor::MOTOR1,          // id 1
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "3508 Motor",
    true,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

// 3510
tap::motor::DjiMotor motor3510(
    drivers(),
    tap::motor::MOTOR4,          // id 4
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "3510 Motor",
    true,
    (1.0f));

// 6020
tap::motor::DjiMotor motor6020(
    drivers(),
    tap::motor::MOTOR7,          // id 3+4
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "6020 Motor",
    true,
    (1.0f));

MotorSubsystem motorSubsystem2006(drivers(), motor2006, m2006VelocityPidConfig);

MotorSubsystem motorSubsystem3505(drivers(), motor3508, rm3508VelocityPidConfig);

MotorSubsystem motorSubsystem6020(drivers(), motor6020, gm6020VelocityPidConfig);

MotorSubsystem motorSubsystem3510(drivers(), motor3510, rm3510VelocityPidConfig);

// ----------
// Commands
// ----------

StickRpmCommand leftVerticalManual(
    &motorSubsystem2006,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_VERTICAL,
    500.0f);

StickRpmCommand leftHorizontalManual(
    &motorSubsystem3510,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL,
    500.0f);

StickRpmCommand rightVerticalManual(
    &motorSubsystem3505,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::RIGHT_VERTICAL,
    482.0f);

StickRpmCommand wheelManual(
    &motorSubsystem6020,
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

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

// inits

void initializeSubsystems()
{
    agitator.initialize();
    motorSubsystem2006.initialize();
    motorSubsystem3505.initialize();
    motorSubsystem6020.initialize();
    motorSubsystem3510.initialize();
}

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &motor_tester_control::remoteSafeDisconnectFunction);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem2006);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem3505);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem6020);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem3510);
}

void registerIoMappings(Drivers* drivers)
{
    drivers->commandMapper.addMap(&leftSwitchUp);

    motorSubsystem6020.setDefaultCommand(&wheelManual);
    motorSubsystem2006.setDefaultCommand(&leftVerticalManual);
    motorSubsystem3505.setDefaultCommand(&rightVerticalManual);
    motorSubsystem3510.setDefaultCommand(&leftHorizontalManual);
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
