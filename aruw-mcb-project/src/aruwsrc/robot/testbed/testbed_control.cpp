/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_TESTBED)

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "aruwsrc/communication/can/aruw_voltage_current_sensor.hpp"
#include "aruwsrc/communication/mcb-lite/virtual_can_encoder.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/chassis/x_drive_chassis_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/robot_control.hpp"
#include "aruwsrc/robot/testbed/testbed_drivers.hpp"

using namespace aruwsrc::testbed;
using namespace aruwsrc::communication::mcb_lite;
using namespace aruwsrc::control::chassis;
using namespace tap::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace testbed_control
{
VirtualCanEncoder forwardEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID0,
    &drivers()->lite,
    tap::can::CanBus::CAN_BUS2);

VirtualCanEncoder strafeEncoder(
    drivers(),
    tap::encoder::CanEncoderId::ID1,
    &drivers()->lite,
    tap::can::CanBus::CAN_BUS2);

aruwsrc::communication::can::AruwVoltageCurrentSensor voltageCurrentSensor(
    drivers(),
    tap::can::CanBus::CAN_BUS2);

tap::motor::DjiMotor leftFrontChassisMotor(
    drivers(),
    aruwsrc::control::chassis::LEFT_FRONT_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Left Front Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor leftBackChassisMotor(
    drivers(),
    aruwsrc::control::chassis::LEFT_BACK_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Left Back Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightFrontChassisMotor(
    drivers(),
    aruwsrc::control::chassis::RIGHT_FRONT_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Right Front Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

tap::motor::DjiMotor rightBackChassisMotor(
    drivers(),
    aruwsrc::control::chassis::RIGHT_BACK_MOTOR_ID,
    aruwsrc::control::chassis::CAN_BUS_MOTORS,
    false,
    "Right Back Chassis Motor",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508);

XDriveChassisSubsystem chassis(
    drivers(),
    &voltageCurrentSensor,
    &voltageCurrentSensor,
    leftFrontChassisMotor,
    leftBackChassisMotor,
    rightFrontChassisMotor,
    rightBackChassisMotor,
    WHEEL_VELOCITY_PID_CONFIG);

// aruwsrc::control::chassis::ChassisImuDriveCommand chassisImuDriveCommand(
//     drivers(),
//     &drivers()->controlOperatorInterface,
//     &chassis,
//     &turret.yawMotor);

aruwsrc::control::chassis::ChassisDriveCommand chassisDriveCommand(
    drivers(),
    &drivers()->controlOperatorInterface,
    &chassis);

// aruwsrc::control::chassis::ChassisAutorotateCommand chassisAutorotateCommand(
//     drivers(),
//     &drivers()->controlOperatorInterface,
//     &chassis,
//     &turret.yawMotor,
//     aruwsrc::control::chassis::ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_180);
// aruwsrc::control::chassis::BeybladeCommand beybladeCommand(
//     drivers(),
//     &chassis,
//     &turret.yawMotor,
//     (drivers()->controlOperatorInterface));

// HoldCommandMapping leftSwitchDown(
//     drivers(),
//     {&beybladeCommand},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));
// HoldCommandMapping leftSwitchUp(
//     drivers(),
//     {&turretCVCommand, &chassisDriveCommand},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// ToggleCommandMapping fToggled(drivers(), {&beybladeCommand}, RemoteMapState({Remote::Key::F}));

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems()
{
    voltageCurrentSensor.initialize();
    chassis.registerAndInitialize();
}

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &testbed_control::remoteSafeDisconnectFunction);
}

void setDefaultCommands(Drivers*) { chassis.setDefaultCommand(&chassisDriveCommand); }

void registerIoMappings(Drivers*)
{
    // drivers->commandMapper.addMap(&leftSwitchDown);
    // drivers->commandMapper.addMap(&leftSwitchUp);
    // drivers->commandMapper.addMap(&fToggled);
}

}  // namespace testbed_control

namespace aruwsrc::testbed
{
void initSubsystemCommands(aruwsrc::testbed::Drivers* drivers)
{
    testbed_control::initializeSubsystems();
    testbed_control::setDefaultCommands(drivers);
    testbed_control::registerIoMappings(drivers);
}

}  // namespace aruwsrc::testbed

#endif
