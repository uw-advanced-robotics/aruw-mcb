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

#ifdef TARGET_TESTBED

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

#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/control/turret/turret_motor_config.hpp"
#include "aruwsrc/robot/hero/hero_turret_subsystem.hpp"
#include "aruwsrc/control/turret/user/turret_user_control_command.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"
#include "aruwsrc/robot/testbed/testbed_constants.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace tap::control;
using namespace aruwsrc::control;
using namespace tap::communication::serial;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;

namespace testbed
{
inline aruwsrc::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
aruwsrc::motor::Tmotor_AK809 legmotorLF(
    drivers(),
    aruwsrc::motor::MOTOR1,
    tap::can::CanBus::CAN_BUS2,
    false,
    "LeftFront Leg");

aruwsrc::motor::Tmotor_AK809 legmotorLR(
    drivers(),
    aruwsrc::motor::MOTOR2,
    tap::can::CanBus::CAN_BUS2,
    false,
    "LeftRear Leg");

// aruwsrc::control::turret::TurretMotor yawMotor(&legmotorLF, YAW_MOTOR_CONFIG);
// aruwsrc::control::turret::TurretMotor pitchMotor(&legmotorLR, PITCH_MOTOR_CONFIG);

aruwsrc::control::turret::HeroTurretSubsystem turret(
    drivers(),
    &legmotorLF,
    &legmotorLR,
    aruwsrc::control::turret::PITCH_MOTOR_CONFIG,
    aruwsrc::control::turret::YAW_MOTOR_CONFIG,
    &getTurretMCBCanComm());

aruwsrc::control::turret::algorithms::
    ChassisFramePitchTurretController chassisFramePitchTurretController(
        turret.pitchMotor,
        TURRET_PID_CONFIG);

aruwsrc::control::turret::algorithms::
    ChassisFrameYawTurretController chassisFrameYawTurretController(
        turret.yawMotor,
        TURRET_PID_CONFIG);

/* define command mappings --------------------------------------------------*/
aruwsrc::control::turret::user::TurretUserControlCommand turretUserCommand(
    drivers(),
    drivers()->controlOperatorInterface,
    &turret,
    &chassisFrameYawTurretController,
    &chassisFramePitchTurretController,
    aruwsrc::control::turret::USER_YAW_INPUT_SCALAR,
    aruwsrc::control::turret::USER_PITCH_INPUT_SCALAR);

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerTestbedSubsystems(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&turret);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() { turret.initialize(); }

/* set any default commands to subsystems here ------------------------------*/
void setDefaultStandardCommands(aruwsrc::Drivers *)
{
    turret.setDefaultCommand(&turretUserCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startStandardCommands(aruwsrc::Drivers *drivers) {}

/* register io mappings here ------------------------------------------------*/
void registerStandardIoMappings(aruwsrc::Drivers *drivers) {}
}  // namespace testbed

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(&testbed::remoteSafeDisconnectFunction);
    testbed::initializeSubsystems();
    testbed::registerTestbedSubsystems(drivers);
    testbed::setDefaultStandardCommands(drivers);
    testbed::startStandardCommands(drivers);
    testbed::registerStandardIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif  // TARGET_TESTBED
