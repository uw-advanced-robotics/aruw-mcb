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
#include <math.h>


// #include "tap/communication/sensors/motor/buzzer.hpp"

namespace aruwsrc::motor_tester
{
MotorSubsystem::MotorSubsystem(tap::Drivers* drivers, tap::motor::MotorInterface* motorInterface, tap::algorithms::SmoothPidConfig config) : 
    Subsystem(drivers), 
    motorInterface(motorInterface),
    desiredPosition(tap::algorithms::WrappedFloat(0, 0, M_TWOPI)),
    pid(config),
    prevTime(0)
    // proportion(50)
{
    
}

void MotorSubsystem::refreshSafeDisconnect() {
    motorInterface->setDesiredOutput(0);
}

void MotorSubsystem::initialize() { 
    motorInterface->initialize();
    motorInterface->setDesiredOutput(0);
    pid.reset();
    // pid.setP(proportion);
}

void MotorSubsystem::refresh() {
    // motorInterface->setDesiredOutput(this->desiredOutput);
    // pid.setP(proportion);

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    tap::algorithms::WrappedFloat posError = desiredPosition - motorInterface->getEncoder()->getPosition();
    float output = pid.runController(posError.getUnwrappedValue(), -motorInterface->getEncoder()->getVelocity(), static_cast<float>(dt) / 1000.0f);

    int32_t value = static_cast<int32_t>(output);
    if (value < -tap::motor::DjiMotor::MAX_OUTPUT_C620) {
        value = -tap::motor::DjiMotor::MAX_OUTPUT_C620;
    }
    if (value > tap::motor::DjiMotor::MAX_OUTPUT_C620) {
        value = tap::motor::DjiMotor::MAX_OUTPUT_C620;
    }

    motorInterface->setDesiredOutput(value);
}

// void MotorSubsystem::setDesiredOutput(int32_t value) {
//     if (value < -tap::motor::DjiMotor::MAX_OUTPUT_C620) {
//         value = -tap::motor::DjiMotor::MAX_OUTPUT_C620;
//     }
//     if (value > tap::motor::DjiMotor::MAX_OUTPUT_C620) {
//         value = tap::motor::DjiMotor::MAX_OUTPUT_C620;
//     }

//     this->desiredOutput = value;
// }

// int32_t MotorSubsystem::getDesiredOutput() { return desiredOutput; }

void MotorSubsystem::setDesiredPosition(tap::algorithms::WrappedFloat pos) {
    this->desiredPosition = pos;
}

tap::algorithms::WrappedFloat MotorSubsystem::getDesiredPosition() const { return desiredPosition; }

bool MotorSubsystem::isOnline() const {
    return motorInterface->isMotorOnline();
}

}  // namespace aruwsrc::control::buzzer
