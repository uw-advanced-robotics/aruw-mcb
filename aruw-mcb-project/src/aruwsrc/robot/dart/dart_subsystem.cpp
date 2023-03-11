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

#include "dart_subsystem.hpp"

#include "tap/control/subsystem.hpp"

namespace aruwsrc::dart
{
DartSubsystem::DartSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* motor1,
                            tap::motor::DjiMotor* motor2, tap::motor::DjiMotor* motor3, tap::motor::DjiMotor* motor4)
    : Subsystem(drivers),
      motor1(motor1),
      motor2(motor2),
      motor3(motor3),
      motor4(motor4),
{
}

void DartSubsystem::windUp() { motor1->setDesiredOutput(WINDUP_SPEED); }

void DartSubsystem::load() 
{
  motor2->setDesiredOutput(LOAD_OUTPUT);
  motor3->setDesiredOutput(LOAD_OUTPUT);
  motor4->setDesiredOutput(LOAD_OUTPUT);
}

void DartSubsystem::stop() { motor1->setDesiredOutput(0); }

}  // namespace aruwsrc::dart
