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

#ifdef TARGET_TESTBED

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
#include "aruwsrc/control/chassis/balancing/balancing_chassis_rel_drive_command.hpp"
#include "aruwsrc/control/chassis/balancing/balancing_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/motion/five_bar_motion_subsystem.hpp"
#include "aruwsrc/control/motion/five_bar_move_command.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"
#include "aruwsrc/robot/testbed/spin_motor_command.hpp"
#include "aruwsrc/robot/testbed/testbed_constants.hpp"
#include "aruwsrc/robot/testbed/testbed_drivers.hpp"
#include "aruwsrc/robot/testbed/tmotor_subsystem.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace tap::control;
using namespace aruwsrc::control;
using namespace aruwsrc::chassis;
using namespace aruwsrc::testbed;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace testbed_control
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

tap::motor::DjiMotor RightWheel(
    drivers(),
    tap::motor::MOTOR6,
    tap::can::CanBus::CAN_BUS1,
    true,
    "Right Wheel Motor");

// aruwsrc::testbed::TMotorSubsystem motorSubsystemLF(drivers(), &legmotorLF);
// aruwsrc::testbed::TMotorSubsystem motorSubsystemLR(drivers(), &legmotorLR);

// aruwsrc::testbed::SpinMotorCommand spinMotorLF(drivers(), &motorSubsystemLF, 500);
// aruwsrc::testbed::SpinMotorCommand spinMotorLR(drivers(), &motorSubsystemLR, -500);

motion::FiveBarLinkage fiveBarLeft(&legmotorLF, &legmotorLR, FIVE_BAR_CONFIG);

motion::FiveBarLinkage fiveBarRight(&legmotorRF, &legmotorRR, FIVE_BAR_CONFIG);

BalancingLeg legLeft(
    drivers(),
    &fiveBarLeft,
    LF_LEG_MOTOR_PID_CONFIG,
    LR_LEG_MOTOR_PID_CONFIG,
    &leftWheel,
    WHEEL_RADIUS,
    LEFT_WHEEL_MOTOR_PID_CONFIG);

BalancingLeg legRight(
    drivers(),
    &fiveBarRight,
    RF_LEG_MOTOR_PID_CONFIG,
    RR_LEG_MOTOR_PID_CONFIG,
    &RightWheel,
    WHEEL_RADIUS,
    RIGHT_WHEEL_MOTOR_PID_CONFIG);

motion::FiveBarMotionSubsystem fiveBarSubsystemLeft(
    drivers(),
    &fiveBarLeft,
    LF_LEG_MOTOR_PID_CONFIG,
    LR_LEG_MOTOR_PID_CONFIG);

motion::FiveBarMotionSubsystem fiveBarSubsystemRight(
    drivers(),
    &fiveBarRight,
    RF_LEG_MOTOR_PID_CONFIG,
    RR_LEG_MOTOR_PID_CONFIG);

aruwsrc::chassis::BalancingChassisSubsystem chassis(drivers(), legLeft, legRight);

BalancingChassisRelativeDriveCommand manualDriveCommand(
    drivers(),
    &chassis,
    drivers()->controlOperatorInterface);

motion::FiveBarMoveCommand moveFiveBarLeftCircle(drivers(), &fiveBarSubsystemLeft, motion::CIRCLE);

motion::FiveBarMoveCommand moveFiveBarLeftSquare(
    drivers(),
    &fiveBarSubsystemLeft,
    motion::UP_AND_DOWN);

motion::FiveBarMoveCommand moveFiveBarRightCircle(
    drivers(),
    &fiveBarSubsystemRight,
    motion::CIRCLE);

motion::FiveBarMoveCommand moveFiveBarRightSquare(
    drivers(),
    &fiveBarSubsystemRight,
    motion::UP_AND_DOWN);

HoldCommandMapping rightSwitchUp(
    drivers(),
    {&moveFiveBarLeftCircle, &moveFiveBarRightCircle},
    RemoteMapState(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::UP));

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&moveFiveBarLeftSquare, &moveFiveBarRightSquare},
    RemoteMapState(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::DOWN));

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerTestbedSubsystems(Drivers *drivers)
{
    // drivers->commandScheduler.registerSubsystem(&motorSubsystemLF);
    // drivers->commandScheduler.registerSubsystem(&motorSubsystemLR);
    drivers->commandScheduler.registerSubsystem(&fiveBarSubsystemLeft);
    drivers->commandScheduler.registerSubsystem(&fiveBarSubsystemRight);
    drivers->commandScheduler.registerSubsystem(&chassis);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    // motorSubsystemLF.initialize();
    // motorSubsystemLR.initialize();
    fiveBarSubsystemLeft.initialize();
    fiveBarSubsystemRight.initialize();
    chassis.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultTestbedCommands(Drivers *) { chassis.setDefaultCommand(&manualDriveCommand); }

/* add any starting commands to the scheduler here --------------------------*/
void startTestbedCommands(Drivers *drivers) {}

/* register io mappings here ------------------------------------------------*/
void registerTestbedIoMappings(Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchDown);
}
}  // namespace testbed_control

namespace aruwsrc::testbed
{
void initSubsystemCommands(Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &testbed_control::remoteSafeDisconnectFunction);
    testbed_control::initializeSubsystems();
    testbed_control::registerTestbedSubsystems(drivers);
    testbed_control::setDefaultTestbedCommands(drivers);
    testbed_control::startTestbedCommands(drivers);
    testbed_control::registerTestbedIoMappings(drivers);
}
}  // namespace aruwsrc::testbed

#endif  // TARGET_TESTBED
