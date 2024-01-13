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

#include "pivot_subsystem.hpp"

namespace aruwsrc::robot::dart
{
PivotSubsystem::PivotSubsystem(
    tap::Drivers* drivers,
    tap::motor::DjiMotor* pivotMotor,
    tap::motor::DjiMotor* pivotDeadMotor,
    const tap::algorithms::SmoothPidConfig& pidParams,
    aruwsrc::control::MotorStallTrigger& trigger1,
    aruwsrc::control::MotorStallTrigger& trigger2)
    : aruwsrc::control::TwoSidedBoundedSubsystemInterface(drivers, trigger1, trigger2),
      drivers(drivers),
      pivotMotor(pivotMotor),
      pivotDeadMotor(pivotDeadMotor),
      pid(pidParams),
      pidParams(pidParams),
      trigger1(trigger1),
      trigger2(trigger2),
      isHomedVector(0),
      lowerBound(0),
      upperBound(0),
      home(0)
{
}

void PivotSubsystem::refresh()
{
    switch(calibrationState) {
        case CalibrationState::AWAITING_CALIBRATE:
            break;
        case CalibrationState::CALIBRATING_LOWER_BOUND:
            // TODO: move towards lower bound, with PID?
            if (trigger1.isTriggered()) {
                calibrationState = CalibrationState::CALIBRATING_UPPER_BOUND;
                stopDuringHoming();
            }
            break;
        case CalibrationState::CALIBRATING_UPPER_BOUND:
            // TODO: move towards upper bound, with PID?
            if (trigger2.isTriggered()) {
                calibrationState = CalibrationState::CALIBRATION_COMPLETE;
                stopDuringHoming();
            }
            break;
        case CalibrationState::CALIBRATION_COMPLETE:
            if (isUsingPID) {
                const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
                const uint32_t dt = curTime - prevTime;
                prevTime = curTime;

                uint64_t pos = pivotDeadMotor->getEncoderUnwrapped();
                uint64_t error = setpoint - pos;
                pid.runControllerDerivateError(error, dt);
                float output = pid.getOutput();

                pivotMotor->setDesiredOutput(output);
            }
            break;
        default:
            //TODO: error
    }
}

void PivotSubsystem::initialize()
{
    pivotMotor->initialize();
    pivotDeadMotor->initialize();
    pivotDeadMotor->setDesiredOutput(0);
}

void PivotSubsystem::setMotor(int32_t motorSpeed)
{
    pivotMotor->setDesiredOutput(motorSpeed);
    isUsingPID = false;
}

void PivotSubsystem::setSetpoint(uint64_t setpoint)
{
    this->setpoint = setpoint;
    isUsingPID = true;
}

void PivotSubsystem::stop() { pivotMotor->setDesiredOutput(0); }

/** Homing functions */
bool PivotSubsystem::homedAndBounded() const {
    uint8_t lowerBoundHomed = isHomedVector & kLowerBoundHomedMask;
    uint8_t upperBoundHomed = isHomedVector & kUpperBoundHomedMask;
    return lowerBoundHomed && upperBoundHomed;
}

uint64_t PivotSubsystem::getUpperBound() const {
    return upperBound;
}

uint64_t PivotSubsystem::getLowerBound() const {
    return lowerBound;
}

void PivotSubsystem::stopDuringHoming() { stop(); }

void PivotSubsystem::setLowerBound(uint64_t encoderPosition) {
    if (isHomedVector & kLowerBoundHomedMask) return;
    lowerBound = encoderPosition;
    isHomedVector |= kLowerBoundHomedMask;
}

void PivotSubsystem::setUpperBound(uint64_t encoderPosition) {
    if (isHomedVector & kUpperBoundHomedMask) return;
    upperBound = encoderPosition;
    isHomedVector |= kUpperBoundHomedMask;
}

void PivotSubsystem::setHome(uint64_t encoderPosition) {
    home = encoderPosition;
}

void PivotSubsystem::moveTowardLowerBound() {
    pivotMotor->setDesiredOutput(0); // TODO: change value, maybe velocity PID?
}

void PivotSubsystem::moveTowardUpperBound() {
    pivotMotor->setDesiredOutput(0); // TODO: change value, maybe velocity PID?
}

}  // namespace aruwsrc::robot::dart
