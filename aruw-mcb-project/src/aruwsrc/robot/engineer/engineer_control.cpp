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

#if defined(TARGET_ENGINEER)

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/arm/arm_lift_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/raw_motor_command.hpp"
#include "aruwsrc/robot/engineer/arm/raw_motor_subsystem.hpp"
#include "aruwsrc/robot/engineer/digital_out_command.hpp"
#include "aruwsrc/robot/engineer/digital_out_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"

using namespace tap::gpio;
using tap::control::CommandMapper;
using namespace aruwsrc::engineer;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace aruwsrc
{
namespace control
{
static constexpr Digital::OutputPin GRABBER_PIN = Digital::OutputPin::E;
static constexpr Digital::OutputPin X_AXIS_PIN = Digital::OutputPin::F;
static constexpr Digital::OutputPin TOWER_LEFT_PIN = Digital::OutputPin::G;
static constexpr Digital::OutputPin TOWER_RIGHT_PIN = Digital::OutputPin::H;
static constexpr Digital::InputPin TOWER_LEFT_LIMIT_SWITCH = Digital::InputPin::B;
static constexpr Digital::InputPin TOWER_RIGHT_LIMIT_SWITCH = Digital::InputPin::C;

/* define subsystems --------------------------------------------------------*/

tap::communication::sensors::current::AnalogCurrentSensor currentSensor(
    {&drivers()->analog,
     aruwsrc::chassis::CURRENT_SENSOR_PIN,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
     aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA});

tap::motor::DjiMotor leftFrontChassisMotor(
    drivers(),
    aruwsrc::chassis::LEFT_FRONT_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Left Front Chassis Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor leftBackChassisMotor(
    drivers(),
    aruwsrc::chassis::LEFT_BACK_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Left Back Chassis Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightFrontChassisMotor(
    drivers(),
    aruwsrc::chassis::RIGHT_FRONT_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Right Front Chassis Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightBackChassisMotor(
    drivers(),
    aruwsrc::chassis::RIGHT_BACK_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Right Back Chassis Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::chassis::MecanumChassisSubsystem chassis(
    drivers(),
    &currentSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    aruwsrc::chassis::WHEEL_VELOCITY_PID_CONFIG);

static constexpr tap::algorithms::SmoothPidConfig LIFT_PID_CONFIG = {
    .kp = 0,
    .ki = 0,
    .kd = 0,
    .maxICumulative = 0,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
};

tap::motor::DoubleDjiMotor liftMotors(
    drivers(),
    tap::motor::MotorId::MOTOR2,
    tap::motor::MotorId::MOTOR1,
    tap::can::CanBus::CAN_BUS1,
    tap::can::CanBus::CAN_BUS1,
    true,
    false,
    "Left Lift Motor",
    "Right Lift Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

aruwsrc::engineer::RawMotorSubsystem lift(drivers(), liftMotors);

aruwsrc::engineer::DigitalOutSubsystem suckSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::E);

aruwsrc::engineer::DigitalOutSubsystem blowSubsystem(
    drivers(),
    drivers()->digital,
    tap::gpio::Digital::OutputPin::F,
    true);

/* define commands ----------------------------------------------------------*/

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

aruwsrc::engineer::RawMotorCommand liftManualCommand(
    &lift,
    &drivers()->remote,
    tap::communication::serial::Remote::Channel::WHEEL,
    5000.0f);

aruwsrc::engineer::DigitalOutCommand suckOffCommand(suckSubsystem, false);
aruwsrc::engineer::DigitalOutCommand suckOnCommand(suckSubsystem, true);
aruwsrc::engineer::DigitalOutCommand blowOffCommand(blowSubsystem, false);
aruwsrc::engineer::DigitalOutCommand blowOnCommand(blowSubsystem, true);

tap::control::HoldCommandMapping leftSwitchDown(
    drivers(),
    {&suckOnCommand},
    tap::control::RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

tap::control::HoldCommandMapping leftSwitchUp(
    drivers(),
    {&blowOnCommand},
    tap::control::RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    lift.initialize();
    suckSubsystem.initialize();
    blowSubsystem.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&lift);
    drivers->commandScheduler.registerSubsystem(&suckSubsystem);
    drivers->commandScheduler.registerSubsystem(&blowSubsystem);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    lift.setDefaultCommand(&liftManualCommand);
    suckSubsystem.setDefaultCommand(&suckOffCommand);
    blowSubsystem.setDefaultCommand(&blowOffCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&leftSwitchUp);
}
}  // namespace control

}  // namespace aruwsrc

namespace aruwsrc::engineer
{
void initSubsystemCommands(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &aruwsrc::control::remoteSafeDisconnectFunction);
    aruwsrc::control::initializeSubsystems();
    aruwsrc::control::registerEngineerSubsystems(drivers);
    aruwsrc::control::setDefaultEngineerCommands(drivers);
    aruwsrc::control::startEngineerCommands(drivers);
    aruwsrc::control::registerEngineerIoMappings(drivers);
}
}  // namespace aruwsrc::engineer

#endif
