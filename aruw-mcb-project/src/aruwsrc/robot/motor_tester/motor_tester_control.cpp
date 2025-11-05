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

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/safe_disconnect.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_constants.hpp"
#include "aruwsrc/robot/motor_tester/motor_tester_drivers.hpp"
#include "aruwsrc/robot/robot_control.hpp"
#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"
#include "aruwsrc/robot/motor_tester/stick_torque_command.hpp"

using namespace aruwsrc::motor_tester;
using namespace aruwsrc::motor_tester::constants;
// using namespace tap::control;

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
    tap::motor::MotorId::MOTOR3, 
    tap::can::CanBus::CAN_BUS1, 
    false, 
    "a"
);

tap::algorithms::SmoothPidConfig pidConfig;

MotorSubsystem motor_subsystem(
    drivers(), 
    motor, 
    pidConfig, 
    0.002
);

float sensitivity = 5;

StickTorqueCommand stick_command(
    motor_subsystem, 
    drivers()->remote,
    tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL, 
    sensitivity
);

// Safe disconnect function
aruwsrc::control::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

void initializeSubsystems() {
    motor_subsystem.initialize();
}

void registerSubsystems(Drivers* drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &motor_tester_control::remoteSafeDisconnectFunction);

    drivers->commandScheduler.registerSubsystem(&motor_subsystem);

    motor_subsystem.setDefaultCommand(&stick_command);
}

void registerIoMappings(Drivers* drivers) {}

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
