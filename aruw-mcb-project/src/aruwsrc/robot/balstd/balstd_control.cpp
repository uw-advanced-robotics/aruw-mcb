/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_beyblade_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_rel_drive_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/motion/five_bar_motion_subsystem.hpp"
#include "aruwsrc/control/motion/five_bar_move_command.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"
#include "aruwsrc/robot/balstd/balstd_drivers.hpp"
#include "aruwsrc/robot/standard/standard_turret_subsystem.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace aruwsrc::control;
using namespace aruwsrc::chassis;
using namespace aruwsrc::balstd;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

namespace balstd_control
{
/* define subsystems --------------------------------------------------------*/
aruwsrc::motor::Tmotor_AK809 legmotorLF(
    drivers(),
    aruwsrc::motor::MOTOR3,
    tap::can::CanBus::CAN_BUS2,
    false,
    "LeftFront Leg");

aruwsrc::motor::Tmotor_AK809 legmotorLR(
    drivers(),
    aruwsrc::motor::MOTOR4,
    tap::can::CanBus::CAN_BUS2,
    false,
    "LeftRear Leg");

aruwsrc::motor::Tmotor_AK809 legmotorRF(
    drivers(),
    aruwsrc::motor::MOTOR1,
    tap::can::CanBus::CAN_BUS2,
    true,
    "RightFront Leg");

aruwsrc::motor::Tmotor_AK809 legmotorRR(
    drivers(),
    aruwsrc::motor::MOTOR2,
    tap::can::CanBus::CAN_BUS2,
    true,
    "RightRear Leg");

tap::motor::DjiMotor leftWheel(
    drivers(),
    tap::motor::MOTOR5,
    tap::can::CanBus::CAN_BUS1,
    false,
    "Left Wheel Motor");

tap::motor::DjiMotor rightWheel(
    drivers(),
    tap::motor::MOTOR6,
    tap::can::CanBus::CAN_BUS1,
    true,
    "Right Wheel Motor");

tap::motor::DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    turret::CAN_BUS_MOTORS,
    false,
    "Pitch Motor");

tap::motor::DjiMotor yawMotor(drivers(), YAW_MOTOR_ID, turret::CAN_BUS_MOTORS, true, "Yaw Motor");

// END HARDWARE INIT

motion::FiveBarLinkage fiveBarLeft(&legmotorLF, &legmotorLR, FIVE_BAR_CONFIG);

motion::FiveBarLinkage fiveBarRight(&legmotorRF, &legmotorRR, FIVE_BAR_CONFIG);

BalancingLeg legLeft(
    drivers(),
    getTurretMCBCanComm(),
    &fiveBarLeft,
    LF_LEG_MOTOR_PID_CONFIG,
    LF_LEG_MOTOR_FUZZY_PID_CONFIG,
    LR_LEG_MOTOR_PID_CONFIG,
    LR_LEG_MOTOR_FUZZY_PID_CONFIG,
    &leftWheel,
    LEFT_WHEEL_MOTOR_PID_CONFIG);

BalancingLeg legRight(
    drivers(),
    getTurretMCBCanComm(),
    &fiveBarRight,
    RF_LEG_MOTOR_PID_CONFIG,
    RF_LEG_MOTOR_FUZZY_PID_CONFIG,
    RR_LEG_MOTOR_PID_CONFIG,
    RR_LEG_MOTOR_FUZZY_PID_CONFIG,
    &rightWheel,
    RIGHT_WHEEL_MOTOR_PID_CONFIG);

// Turret controllers
// algorithms::ChassisFramePitchTurretController chassisFramePitchTurretController(
//     turret.pitchMotor,
//     chassis_rel::PITCH_PID_CONFIG);

// algorithms::ChassisFrameYawTurretController chassisFrameYawTurretController(
//     turret.yawMotor,
//     chassis_rel::YAW_PID_CONFIG);

tap::algorithms::SmoothPid worldFramePitchTurretImuPosPid(
    turret::world_rel_turret_imu::PITCH_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuPosPidCv(
    turret::world_rel_turret_imu::PITCH_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFramePitchTurretImuVelPid(
    turret::world_rel_turret_imu::PITCH_VEL_PID_CONFIG);

turret::algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController(
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretImuPosPid,
        worldFramePitchTurretImuVelPid);

turret::algorithms::
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuControllerCv(
        getTurretMCBCanComm(),
        turret.pitchMotor,
        worldFramePitchTurretImuPosPidCv,
        worldFramePitchTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPid(
    turret::world_rel_turret_imu::YAW_POS_PID_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPid(
    turret::world_rel_turret_imu::YAW_VEL_PID_CONFIG);

turret::algorithms::
    WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(
        getTurretMCBCanComm(),
        turret.yawMotor,
        worldFrameYawTurretImuPosPid,
        worldFrameYawTurretImuVelPid);

tap::algorithms::SmoothPid worldFrameYawTurretImuPosPidCv(
    turret::world_rel_turret_imu::YAW_POS_PID_AUTO_AIM_CONFIG);
tap::algorithms::SmoothPid worldFrameYawTurretImuVelPidCv(
    turret::world_rel_turret_imu::YAW_VEL_PID_CONFIG);

turret::algorithms::
    WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuControllerCv(
        getTurretMCBCanComm(),
        turret.yawMotor,
        worldFrameYawTurretImuPosPidCv,
        worldFrameYawTurretImuVelPidCv);

// BEGIN SUBSYSTEMS

aruwsrc::control::launcher::RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE>
    frictionWheels(
        drivers(),
        aruwsrc::control::launcher::LEFT_MOTOR_ID,
        aruwsrc::control::launcher::RIGHT_MOTOR_ID,
        aruwsrc::control::launcher::CAN_BUS_MOTORS,
        &getTurretMCBCanComm(),
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

aruwsrc::chassis::BalancingChassisSubsystem chassis(
    drivers(),
    getTurretMCBCanComm(),
    legLeft,
    legRight);

aruwsrc::control::turret::StandardTurretSubsystem turret(
    drivers(),
    &pitchMotor,
    &yawMotor,
    PITCH_MOTOR_CONFIG,
    YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

BalancingChassisRelativeDriveCommand manualDriveCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface);

BalancingChassisAutorotateCommand autorotateDriveCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface,
    &turret.yawMotor);

BalancingChassisAutorotateCommand autorotateDriveCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface,
    &turret.yawMotor);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerTestbedSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() { chassis.initialize(); }

/* set any default commands to subsystems here ------------------------------*/
void setDefaultTestbedCommands(Drivers *) { chassis.setDefaultCommand(&manualDriveCommand); }

/* add any starting commands to the scheduler here --------------------------*/
void startTestbedCommands(Drivers *drivers) {}

/* register io mappings here ------------------------------------------------*/
void registerTestbedIoMappings(Drivers *drivers) {}
}  // namespace balstd_control

namespace aruwsrc::balstd
{
void initSubsystemCommands(Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &balstd_control::remoteSafeDisconnectFunction);
    balstd_control::initializeSubsystems();
    balstd_control::registerTestbedSubsystems(drivers);
    balstd_control::setDefaultTestbedCommands(drivers);
    balstd_control::startTestbedCommands(drivers);
    balstd_control::registerTestbedIoMappings(drivers);
}
}  // namespace aruwsrc::balstd

#endif  // TARGET_BALSTD
