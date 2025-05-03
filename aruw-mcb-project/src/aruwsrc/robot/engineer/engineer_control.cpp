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
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/control/command_scheduler.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/engineer/arm/arm_extension_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/arm_lift_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/joint_subsystem.hpp"
#include "aruwsrc/robot/engineer/arm/wrist_subsystem.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/engineer/engineer_gantry_constants.hpp"

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

// aruwsrc::chassis::MecanumChassisSubsystem chassis(
//     drivers(),
//     &currentSensor,
//     leftFrontChassisMotor,
//     leftBackChassisMotor,
//     rightFrontChassisMotor,
//     rightBackChassisMotor,
//     aruwsrc::chassis::WHEEL_VELOCITY_PID_CONFIG);

tap::motor::DjiMotor engineerWristRollMotor(
    drivers(),
    aruwsrc::engineer::WRIST_ROLL_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    " Wrist Roll Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor engineerWristLeftMotor(
    drivers(),
    aruwsrc::engineer::WRIST_LEFT_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Wrist Left Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor engineerWristRightMotor(
    drivers(),
    aruwsrc::engineer::WRIST_RIGHT_MOTOR_ID,
    aruwsrc::chassis::CAN_BUS_MOTORS,
    false,
    "Wrist Right Motor",
    false,
    1.0f / tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

/* define commands ----------------------------------------------------------*/

// aruwsrc::chassis::ChassisDriveCommand chassisDriveCommand(
//     drivers(),
//     &drivers()->controlOperatorInterface,
//     &chassis);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() { /*chassis.initialize();*/ }

/* register subsystems here -------------------------------------------------*/
void registerEngineerSubsystems(aruwsrc::engineer::Drivers *drivers)
{
    // drivers->commandScheduler.registerSubsystem(&chassis);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultEngineerCommands(aruwsrc::engineer::Drivers *)
{
    // chassis.setDefaultCommand(&chassisDriveCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startEngineerCommands(aruwsrc::engineer::Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerEngineerIoMappings(aruwsrc::engineer::Drivers *) {}
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
