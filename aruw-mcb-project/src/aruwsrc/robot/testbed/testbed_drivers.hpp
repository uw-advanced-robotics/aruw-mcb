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

#ifndef TESTBED_DRIVERS_HPP_
#define TESTBED_DRIVERS_HPP_

#include "tap/drivers.hpp"

namespace aruwsrc::testbed
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
          motor1(this, motor1ID, tap::can::CanBus::CAN_BUS1, false, "Testbed spinny motor 1"),
          motor2(this, motor2ID, tap::can::CanBus::CAN_BUS1, false, "Testbed spinny motor 2")
    {
    }

public:
        // Motors you just want to spin
    tap::motor::MotorId motor1ID = tap::motor::MotorId::MOTOR1;
    tap::motor::MotorId motor2ID = tap::motor::MotorId::MOTOR2;

    tap::motor::DjiMotor motor1;
    tap::motor::DjiMotor motor2;

};  // class aruwsrc::StandardDrivers
}  // namespace aruwsrc::testbed

#endif  // STANDARD_DRIVERS_HPP_
