/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/robot_control.hpp"
#include "aruwsrc/robot/testbed/testbed_drivers.hpp"
#include "aruwsrc/robot/testbed/motor_subsystem.hpp"

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

tap::motor::DjiMotor leftChannelMotor(drivers(),
    tap::motor::MOTOR1,  // id 1
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "LMotor");
MotorSubsystem leftMotorSubsystem(drivers(), leftChannelMotor, tap::communication::serial::Remote::Channel::LEFT_VERTICAL, static_cast<int16_t>(10000));

tap::motor::DjiMotor rightChannelMotor(drivers(),
    tap::motor::MOTOR2,  // id 2
    tap::can::CanBus::CAN_BUS1,  // bus 1
    false,
    "RMotor");
MotorSubsystem rightMotorSubsystem(drivers(), rightChannelMotor, tap::communication::serial::Remote::Channel::RIGHT_VERTICAL, static_cast<int16_t>(10000));

void initializeSubsystems() {
    leftMotorSubsystem.initialize();
    rightMotorSubsystem.initialize();
}

void registerSubsystems() {
    drivers()->commandScheduler.registerSubsystem(&leftMotorSubsystem);
    drivers()->commandScheduler.registerSubsystem(&rightMotorSubsystem);
}
}  // namespace testbed_control

namespace aruwsrc::testbed
{
void initSubsystemCommands(aruwsrc::testbed::Drivers *drivers)
{
    testbed_control::registerSubsystems();
    testbed_control::initializeSubsystems();
}

}  // namespace aruwsrc::testbed

#endif
