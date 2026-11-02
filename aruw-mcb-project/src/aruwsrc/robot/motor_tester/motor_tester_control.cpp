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

#include <vector>

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"
#if defined(TARGET_MOTOR_TESTER)
#include <memory>

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/instant_command.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/unjam_spoke_agitator_command.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/motor/damiao_motor.hpp"
#include "aruwsrc/control/motor/damiao_subsystem.hpp"
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

aruwsrc::control::motor::DamiaoMotor damiao(
    drivers(),
    aruwsrc::control::motor::DamiaoMotorId::DAMIAO_MOTOR1,
    tap::can::CanBus::CAN_BUS1,
    false,
    "Damiao Motor",
    0x011,
    aruwsrc::control::motor::DAMIAO_CONTROL_MODE_VELOCITY);
aruwsrc::control::motor::DamiaoSubsystem damiaoSubsystem(drivers(), damiao);

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
    tap::motor::DjiMotorEncoder::GEAR_RATIO_GM6020);

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
tap::control::Subsystem dummySubsystem(drivers());

auto leftSwitchUpRms = tap::control::RemoteMapState(
    tap::communication::serial::Remote::Switch::LEFT_SWITCH,
    tap::communication::serial::Remote::SwitchState::UP);
auto leftSwitchUp = std::make_unique<tap::control::HoldRepeatCommandMapping>(
    drivers(),
    std::vector<tap::control::Command*>{&rotateAndUnjamAgitator},
    &leftSwitchUpRms,
    true);

tap::control::InstantCommand<1> sendMitCommand(
    []() { damiaoSubsystem.sendMitCommand(0.0, 3.0, 5.0, 0.01, 3.0); },
    std::array<tap::control::Subsystem*, 1>{&damiaoSubsystem}

);
tap::control::InstantCommand<1> sendVelo(
    []() { damiaoSubsystem.sendVeloCommand(1.0); },
    std::array<tap::control::Subsystem*, 1>{&damiaoSubsystem}

);
tap::control::InstantCommand<1> stopDamiao(
    []() { damiaoSubsystem.stop(); },
    std::array<tap::control::Subsystem*, 1>{&damiaoSubsystem});
auto rightSwitchUpRms = tap::control::RemoteMapState(
    tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
    tap::communication::serial::Remote::SwitchState::UP);

tap::control::Trigger rightSwitchUp = tap::control::TriggerHelpers::switchState(
                                          drivers(),
                                          Remote::Switch::RIGHT_SWITCH,
                                          Remote::SwitchState::UP)
                                          .whileTrue(&sendMitCommand);
tap::control::Trigger rightSwitchMid = tap::control::TriggerHelpers::switchState(
                                           drivers(),
                                           Remote::Switch::RIGHT_SWITCH,
                                           Remote::SwitchState::MID)
                                           .whileTrue(&stopDamiao);
tap::control::Trigger rightSwitchDown = tap::control::TriggerHelpers::switchState(
                                            drivers(),
                                            Remote::Switch::RIGHT_SWITCH,
                                            Remote::SwitchState::DOWN)
                                            .whileTrue(&sendVelo);

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
    damiaoSubsystem.initialize();
}

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &motor_tester_control::remoteSafeDisconnectFunction);
    // drivers->commandScheduler.registerSubsystem(&motorSubsystem2006);
    drivers->commandScheduler.registerSubsystem(&agitator);
    drivers->commandScheduler.registerSubsystem(&damiaoSubsystem);
    // drivers->commandScheduler.registerSubsystem(&motorSubsystem3505);
    // drivers->commandScheduler.registerSubsystem(&motorSubsystem6020);
    // drivers->commandScheduler.registerSubsystem(&motorSubsystem3510);
}

void registerIoMappings(Drivers* drivers)
{
    // motorSubsystem6020.setDefaultCommand(&wheelManual);
    // motorSubsystem2006.setDefaultCommand(&leftVerticalManual);
    // motorSubsystem3505.setDefaultCommand(&rightVerticalManual);
    // motorSubsystem3510.setDefaultCommand(&leftHorizontalManual);
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
