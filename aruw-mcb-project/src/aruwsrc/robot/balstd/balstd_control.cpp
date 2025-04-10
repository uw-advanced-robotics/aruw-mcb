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

#include "aruwsrc/util_macros.hpp"

#ifdef TARGET_BALSTD

#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/buzzer/buzzer_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
#include "aruwsrc/display/imu_calibrate_menu.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/balstd/balstd_drivers.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_subsystem.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_leg.hpp"
#include "aruwsrc/robot/balstd/chassis/controllers/manual_leg_controller.hpp"
#include "aruwsrc/robot/balstd/fsm/balstd_op_state_machine.hpp"
#include "aruwsrc/robot/balstd/turret/balstd_turret_subsystem.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace aruwsrc::balstd;
using namespace aruwsrc::control;
using namespace aruwsrc::control::balstd;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::algorithms;
// using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms::transforms;
using namespace tap::control::setpoint;
using namespace tap::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace balstd_control
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
tap::motor::DjiMotor leftFrontHipMotor(
    drivers(),
    tap::motor::MotorId::MOTOR1,
    tap::can::CanBus::CAN_BUS2,
    false,
    "left front hip");
tap::motor::DjiMotor leftBackHipMotor(
    drivers(),
    tap::motor::MotorId::MOTOR2,
    tap::can::CanBus::CAN_BUS2,
    false,
    "left back hip");
tap::motor::DjiMotor leftWheelMotor(
    drivers(),
    tap::motor::MotorId::MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    false,
    "left wheel");

tap::motor::DjiMotor rightFrontHipMotor(
    drivers(),
    tap::motor::MotorId::MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    true,
    "right front hip");
tap::motor::DjiMotor rightBackHipMotor(
    drivers(),
    tap::motor::MotorId::MOTOR5,
    tap::can::CanBus::CAN_BUS2,
    true,
    "right back hip");
tap::motor::DjiMotor rightWheelMotor(
    drivers(),
    tap::motor::MotorId::MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    true,
    "right wheel");

BalstdLegConfig legConfig{
    .upperLinkLength = 0.15,
    .lowerLinkLength = 0.25,
    .fixedLinkLength = 0.108,
    .frontHipOuterLimit = modm::toRadian(-15),
    .frontHipInnerLimit = modm::toRadian(90),
    .backHipOuterLimit = modm::toRadian(195),
    .backHipInnerLimit = modm::toRadian(90),
};

BalstdLeg leftLeg(leftFrontHipMotor, leftBackHipMotor, leftWheelMotor, legConfig);
BalstdLeg rightLeg(rightFrontHipMotor, rightBackHipMotor, rightWheelMotor, legConfig);

BalstdChassisSubsystem chassis(drivers(), leftLeg, rightLeg);

// controllers

ManualLegController manualLegController(drivers()->controlOperatorInterface);

// transforms
// StandardAndHeroTransformer transformer(odometrySubsystem, turret);
// StandardAnderHeroTransformerSubsystem transformSubsystem(*drivers(), transformer);

// StandardAndHeroTransformAdapter transformAdapter(transformer);

BalstdOpStateMachine stateMachine(drivers(), chassis.getChassisState());

// turret
tap::motor::DjiMotor pitchMotor(
    drivers(),
    aruwsrc::control::turret::PITCH_MOTOR_ID,
    aruwsrc::control::turret::CAN_BUS_PITCH_MOTOR,
    false,
    "Pitch Turret",
    true,
    PITCH_MOTOR_CONFIG.startEncoderValue);

tap::motor::DjiMotor yawMotor(
    drivers(),
    aruwsrc::control::turret::YAW_MOTOR_ID,
    aruwsrc::control::turret::CAN_BUS_YAW_MOTOR,
    true,
    "Yaw Turret",
    true);

aruwsrc::control::turret::BalstdTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
    turret.pitchMotor,
    chassis_rel::PITCH_PID_CONFIG);

algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
    turret.yawMotor,
    chassis_rel::YAW_PID_CONFIG);

user::TurretUserControlCommand turretUserControlCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    USER_YAW_INPUT_SCALAR,
    USER_PITCH_INPUT_SCALAR,
    0  // Assuming this is the desired turret ID
);

/* define commands ----------------------------------------------------------*/

// aruwsrc::control::buzzer::BuzzerSubsystem buzzer(drivers());

/* define command mappings --------------------------------------------------*/

// Remote related mappings

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerStandardSubsystems(Drivers *drivers)
{
    // drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    // drivers->commandScheduler.registerSubsystem(&stateMachine);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    // chassis.initialize();
    // stateMachine.initialize();
    turret.initialize();
    // odometrySubsystem.initialize();
    // transformSubsystem.initialize();

    // chassis.attachController(&manualLegController);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(Drivers *)
{
    turret.setDefaultCommand(&turretUserControlCommand);
    // chassis.setDefaultCommand(&chassisAutorotateCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(Drivers *)
{
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
    // drivers->commandScheduler.addCommand(&imuCalibrateCommand);
    // drivers->visionCoprocessor.attachTransformer(&transformAdapter);
    // drivers->commandScheduler.addCommand(&turretUserControlCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(Drivers *) {}
}  // namespace balstd_control

namespace aruwsrc::balstd
{
void initSubsystemCommands(aruwsrc::balstd::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &balstd_control::remoteSafeDisconnectFunction);
    balstd_control::initializeSubsystems();
    balstd_control::registerStandardSubsystems(drivers);
    balstd_control::setDefaultStandardCommands(drivers);
    balstd_control::startStandardCommands(drivers);
    balstd_control::registerStandardIoMappings(drivers);
}
}  // namespace aruwsrc::balstd

#ifndef PLATFORM_HOSTED
imu::ImuCalibrateCommand *getImuCalibrateCommand()
{
    return nullptr;  //&balstd_control::imuCalibrateCommand;
}
#endif

#endif
