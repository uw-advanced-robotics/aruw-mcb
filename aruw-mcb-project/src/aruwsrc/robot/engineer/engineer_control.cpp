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

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/hold_command_mapping.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/bounded-subsystem/homing_command.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/limit_switch_trigger.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/cube_lift/cube_move_manual_command.hpp"
#include "aruwsrc/robot/engineer/cube_lift/cube_move_position_command.hpp"
#include "aruwsrc/robot/engineer/cube_lift/cube_storage_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
using namespace tap::gpio;
using tap::communication::serial::Remote;
using tap::control::CommandMapper;
using namespace aruwsrc::engineer;
using namespace aruwsrc::robot::engineer;
using namespace tap::control;

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
tap::motor::DjiMotor storageLiftMotor(
    drivers(),
    CUBE_LIFT_MOTOR_ID,
    LIFT_MOTOR_CAN_BUS,
    false,
    "Lifting Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);
LimitSwitchTrigger cubeLiftTrigger(drivers(), CUBELIFT_LIMITSWITCH_PORT);
/* define subsystems --------------------------------------------------------*/
CubeStorageSubsystem cubeLift(drivers(), storageLiftMotor, cubeLiftTrigger, LENGTH);

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

/* define commands ----------------------------------------------------------*/
HomingCommand cubeLiftHome(cubeLift);

// HomingCommand cubeHomingCommand(cubeLift);

CubeMoveManualCommand cubeManualControl(cubeLift, &drivers()->controlOperatorInterface);
CubeMovePositionCommand oneCubePosition(cubeLift, ONE_CUBE_SETPOINT);
CubeMovePositionCommand twoCubePosition(cubeLift, TWO_CUBE_SETPOINT);
CubeMovePositionCommand threeCubePosition(cubeLift, THREE_CUBE_SETPOINT);

aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

tap::control::HoldCommandMapping leftSwitchUp(
    drivers(),
    {&cubeLiftHome},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

tap::control::HoldCommandMapping leftSwitchMid(
    drivers(),
    {&cubeManualControl},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

tap::control::HoldCommandMapping leftDownRightUp(
    drivers(),
    {&threeCubePosition},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::UP));
tap::control::HoldCommandMapping leftDownRightMid(
    drivers(),
    {&twoCubePosition},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::MID));

tap::control::HoldCommandMapping leftDownRightDown(
    drivers(),
    {&oneCubePosition},
    RemoteMapState(Remote::SwitchState::DOWN, Remote::SwitchState::DOWN));

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    cubeLift.initialize();
}

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&cubeLift);
    drivers->commandScheduler.registerSubsystem(&chassis);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *drivers)
{
    // drivers->commandMapper.addMap(&rightSwitchUp);
    // drivers->commandMapper.addMap(&rightSwitchDown); // old manual power commands
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftDownRightUp);
    drivers->commandMapper.addMap(&leftDownRightMid);
    drivers->commandMapper.addMap(&leftDownRightDown);
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
