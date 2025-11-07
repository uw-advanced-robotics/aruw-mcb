/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_MOTOR_TESTER)

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_constants.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_drivers.hpp"
#include "aruwsrc/robot/motor_tester/stick_position_command.hpp"
#include "aruwsrc/robot/motor_tester/stick_torque_command.hpp"
#include "aruwsrc/robot/robot_control.hpp"
// using namespace tap::control;

using namespace aruwsrc::motor_tester;
using namespace aruwsrc::robot::motor_tester;
using namespace aruwsrc::control;
/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace motor_tester_control
{
// motors, subsystems, commands, etc.

tap::motor::DjiMotor motor(
    drivers(),
    tap::motor::MotorId::MOTOR3,  // motor id
    tap::can::CanBus::CAN_BUS1,   // can bus
    false,
    "MOTOR");
MotorSubsystem motorsubsystem(drivers(), motor, tap::algorithms::SmoothPidConfig());

// StickPositionCommand stickPositionCommand(
//     drivers(),
//     tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL,
//     &motorsubsystem,
//     1000.0f);

StickTorqueCommand stickTorqueCommand(
    drivers(),
    tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL,
    &motorsubsystem,
    0.5f);

void initializeSubsystems() { motorsubsystem.initialize(); }

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&motorsubsystem);
    motorsubsystem.setDefaultCommand(&stickTorqueCommand);
}

void registerIoMappings(Drivers*) {}

}  // namespace motor_tester_control

namespace aruwsrc::motor_tester
{
void initSubsystemCommands(aruwsrc::motor_tester::Drivers* drivers)
{
    motor_tester_control::registerSubsystems(drivers);
    motor_tester_control::initializeSubsystems();
    motor_tester_control::registerIoMappings(drivers);
}

}  // namespace aruwsrc::motor_tester

#endif
