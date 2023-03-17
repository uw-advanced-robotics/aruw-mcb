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

#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/control/motion/five_bar_motion_subsystem.hpp"
#include "aruwsrc/control/motion/five_bar_move_command.hpp"
#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/motor/tmotor_ak80-9.hpp"
#include "aruwsrc/robot/testbed/spin_motor_command.hpp"
#include "aruwsrc/robot/testbed/testbed_constants.hpp"
#include "aruwsrc/robot/testbed/tmotor_subsystem.hpp"

#ifdef PLATFORM_HOSTED
#include "tap/communication/can/can.hpp"
#endif

using namespace tap::control::setpoint;
using namespace tap::control::governor;
using namespace tap::control;
using namespace aruwsrc::control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
aruwsrc::driversFunc drivers = aruwsrc::DoNotUse_getDrivers;
using namespace aruwsrc::control;
using namespace aruwsrc::chassis;

namespace testbed_control
{

/* define subsystems --------------------------------------------------------*/
aruwsrc::motor::Tmotor_AK809 legmotorLF(
    drivers(),
    aruwsrc::motor::MOTOR2,
    tap::can::CanBus::CAN_BUS2,
    true,
    "LeftFront Leg");

aruwsrc::motor::Tmotor_AK809 legmotorLR(
    drivers(),
    aruwsrc::motor::MOTOR1,
    tap::can::CanBus::CAN_BUS2,
    true,
    "LeftRear Leg");

// aruwsrc::testbed::TMotorSubsystem motorSubsystemLF(drivers(), &legmotorLF);
// aruwsrc::testbed::TMotorSubsystem motorSubsystemLR(drivers(), &legmotorLR);

// aruwsrc::testbed::SpinMotorCommand spinMotorLF(drivers(), &motorSubsystemLF, 500);
// aruwsrc::testbed::SpinMotorCommand spinMotorLR(drivers(), &motorSubsystemLR, -500);

motion::FiveBarMotionSubsystem fiveBarSubsystemLeft(
    drivers(),
    &legmotorLF,
    &legmotorLR,
    FIVE_BAR_CONFIG,
    LEG_MOTOR_PID_CONFIG);

motion::FiveBarMoveCommand moveFiveBarLeftCircle(drivers(), &fiveBarSubsystemLeft, motion::SQUARE);

motion::FiveBarMoveCommand moveFiveBarLeftSquare(
    drivers(),
    &fiveBarSubsystemLeft,
    motion::UP_AND_DOWN);

HoldCommandMapping rightSwitchUp(
    drivers(),
    {&moveFiveBarLeftCircle},
    RemoteMapState(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::UP));

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&moveFiveBarLeftSquare},
    RemoteMapState(
        tap::communication::serial::Remote::Switch::RIGHT_SWITCH,
        tap::communication::serial::Remote::SwitchState::DOWN));

// Safe disconnect function
RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* register subsystems here -------------------------------------------------*/
void registerTestbedSubsystems(aruwsrc::Drivers *drivers)
{
    // drivers->commandScheduler.registerSubsystem(&motorSubsystemLF);
    // drivers->commandScheduler.registerSubsystem(&motorSubsystemLR);
    drivers->commandScheduler.registerSubsystem(&fiveBarSubsystemLeft);
}

/* initialize subsystems ----------------------------------------------------*/
void initializeSubsystems()
{
    // motorSubsystemLF.initialize();
    // motorSubsystemLR.initialize();
    fiveBarSubsystemLeft.initialize();
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultTestbedCommands(aruwsrc::Drivers *) {}

/* add any starting commands to the scheduler here --------------------------*/
void startTestbedCommands(aruwsrc::Drivers *drivers) {}

/* register io mappings here ------------------------------------------------*/
void registerTestbedIoMappings(aruwsrc::Drivers *drivers)
{
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchDown);
}
}  // namespace testbed_control

namespace aruwsrc::control
{
void initSubsystemCommands(aruwsrc::Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &testbed_control::remoteSafeDisconnectFunction);
    testbed_control::initializeSubsystems();
    testbed_control::registerTestbedSubsystems(drivers);
    testbed_control::setDefaultTestbedCommands(drivers);
    testbed_control::startTestbedCommands(drivers);
    testbed_control::registerTestbedIoMappings(drivers);
}
}  // namespace aruwsrc::control

#endif  // TARGET_TESTBED
