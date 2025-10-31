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

#include "MotorSubsystem.hpp"

// #include "tap/communication/sensors/motor/buzzer.hpp"

namespace aruwsrc::motor_tester
{
MotorSubsystem::MotorSubsystem(tap::Drivers* drivers, tap::motor::MotorInterface* motorInterface) : 
    Subsystem(drivers), 
    motorInterface(motorInterface)
{

}

void MotorSubsystem::refreshSafeDisconnect() {
    motorInterface->setDesiredOutput(0);
}

void MotorSubsystem::initialize() { 
    motorInterface->setDesiredOutput(0);
}

void MotorSubsystem::refresh() {
    motorInterface->setDesiredOutput(this->desiredOutput);
}

void MotorSubsystem::setDesiredOutput(int32_t value) {
    if (value < -tap::motor::DjiMotor::MAX_OUTPUT_C620) {
        value = -tap::motor::DjiMotor::MAX_OUTPUT_C620;
    }
    if (value > tap::motor::DjiMotor::MAX_OUTPUT_C620) {
        value = tap::motor::DjiMotor::MAX_OUTPUT_C620;
    }

    this->desiredOutput = value;
}

int32_t MotorSubsystem::getDesiredOutput() { return desiredOutput; }

bool MotorSubsystem::isOnline() const {
    return motorInterface->isMotorOnline();
}

}  // namespace aruwsrc::control::buzzer
