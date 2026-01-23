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

#if defined(TARGET_FLYWHEEL_TESTING)

#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/motor/double_dji_motor.hpp"

#include "aruwsrc/control/launcher/friction_wheel_interface.hpp"
#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace tap::communication::serial;
using namespace tap::control;
using namespace tap::control::governor;
using namespace aruwsrc::algorithms;
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::algorithms::odometry::transforms;
using namespace aruwsrc::control::chassis;
using namespace aruwsrc::control;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::flywheel_testing;
using tap::control::CommandMapper;
using tap::control::RemoteMapState;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace flywheel_testing_control
{
inline aruwsrc::communication::can::TurretMCBCanComm &getTurretMCBCanComm()
{
    return drivers()->turretMCBCanCommBus1;
}

/* define subsystems --------------------------------------------------------*/
tap::motor::DjiMotor leftFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::LEFT_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    true,
    "Left flywheel");
tap::motor::DjiMotor rightFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::RIGHT_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    false,
    "Right flywheel");
tap::motor::DjiMotor upperFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::UPPER_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    true,
    "Upper flywheel");
tap::motor::DjiMotor lowerFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::LOWER_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    false,
    "Lower flywheel");
tap::motor::DjiMotor upperSmallFrictionWheel(
    drivers(),
    aruwsrc::control::launcher::UPPER_SMALL_MOTOR_ID,
    aruwsrc::control::launcher::CAN_BUS_MOTORS,
    true,
    "Upper small flywheel",
    false,
    tap::motor::DjiMotorEncoder::GEAR_RATIO_M2006);
std::array<tap::motor::MotorInterface *, 5> wheels = {
    &leftFrictionWheel,
    &rightFrictionWheel,
    &lowerFrictionWheel,
    &upperFrictionWheel,
    &upperSmallFrictionWheel};
RefereeFeedbackFrictionWheelSubsystem<
    aruwsrc::control::launcher::LAUNCH_SPEED_AVERAGING_DEQUE_SIZE,
    5>
    frictionWheelsSubsystem(
        drivers(),
        wheels,
        aruwsrc::control::launcher::WHEEL_CONFIG,
        &getTurretMCBCanComm(),
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

FrictionWheelInterface &frictionWheels = frictionWheelsSubsystem;

/* define commands ----------------------------------------------------------*/

FrictionWheelSpinRefLimitedCommand spinFrictionWheels(
    drivers(),
    &frictionWheels,
    14.0f,
    false,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

FrictionWheelSpinRefLimitedCommand stopFrictionWheels(
    drivers(),
    &frictionWheels,
    0.0f,
    true,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

/* define command mappings --------------------------------------------------*/
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&spinFrictionWheels},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems() { frictionWheels.initialize(); }

/* register subsystems here -------------------------------------------------*/
void registerHeroSubsystems(Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultHeroCommands() { frictionWheels.setDefaultCommand(&stopFrictionWheels); }

/* add any starting commands to the scheduler here --------------------------*/
void startHeroCommands(Drivers *) {}

/* register io mappings here ------------------------------------------------*/
void registerHeroIoMappings(Drivers *drivers) { drivers->commandMapper.addMap(&rightSwitchUp); }
}  // namespace flywheel_testing_control

namespace aruwsrc::flywheel_testing
{
void initSubsystemCommands(aruwsrc::flywheel_testing::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &flywheel_testing_control::remoteSafeDisconnectFunction);
    flywheel_testing_control::initializeSubsystems();
    flywheel_testing_control::registerHeroSubsystems(drivers);
    flywheel_testing_control::setDefaultHeroCommands();
    flywheel_testing_control::startHeroCommands(drivers);
    flywheel_testing_control::registerHeroIoMappings(drivers);
}
}  // namespace aruwsrc::flywheel_testing

#endif
