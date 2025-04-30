/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "cube_storage_subsystem.hpp"

#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::robot::engineer
{
CubeStorageSubsystem::CubeStorageSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& storageLiftMotor,
    aruwsrc::control::TriggerInterface& trigger,
    uint64_t length)
    : OneSidedBoundedSubsystemInterface(drivers, trigger, length),
      motor(storageLiftMotor){ calibrationState = CalibrationState::AWAITING_CALIBRATE; };

void CubeStorageSubsystem::initialize()
{
    motor.initialize();
    moveMotor(0);
}

void CubeStorageSubsystem::moveMotor(int16_t power) { motorDesiredOutput = power + FEEDFORWARD; }

bool CubeStorageSubsystem::homedAndBounded() const
{
    return calibrationState == CalibrationState::CALIBRATION_COMPLETE;
}

void CubeStorageSubsystem::stopDuringHoming()
{
    isPIDControl = false;
    moveMotor(0);
}

void CubeStorageSubsystem::setSetpoint(float newSetpoint) { setpoint = newSetpoint; }

float CubeStorageSubsystem::getSetpoint() { return setpoint; }

void CubeStorageSubsystem::moveTowardLowerBound()
{
    isPIDControl = false;
    moveMotor(homingOutput);
}

void CubeStorageSubsystem::setHome(uint64_t encoderPosition) { home = encoderPosition; }

void CubeStorageSubsystem::setUpperBound(uint64_t encoderPosition) { upperBound = encoderPosition; }

uint64_t CubeStorageSubsystem::getUpperBound() const { return upperBound; }

void CubeStorageSubsystem::setLowerBound(uint64_t encoderPosition) { lowerBound = encoderPosition; }

uint64_t CubeStorageSubsystem::getLowerBound() const { return lowerBound; }

void CubeStorageSubsystem::refreshSafeDisconnect()
{
    isPIDControl = false;
    motor.setDesiredOutput(0);
}

bool CubeStorageSubsystem::isLimitSwitched()
{
    return !drivers->digital.read(CUBELIFT_LIMITSWITCH_PORT);
}

void CubeStorageSubsystem::refresh()
{
    
    if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
    {
        
        if (!trigger.isTriggered())
        {
            calibrationState = CalibrationState::CALIBRATION_COMPLETE;
            motor.getEncoder()->resetEncoderValue();
            moveMotor(0);
            isLimitSwitch = isLimitSwitched();
        }
        else
        {
            moveTowardLowerBound();
        }
        
    }
    

    if (isPIDControl)
    {
        float error =
            setpoint - motor.getEncoder()->getPosition().getUnwrappedValue() / MM_PER_REVOLUTION;
        float errorDerivative =
            motor.getEncoder()->getVelocity() / 1000 / 60 / MM_PER_REVOLUTION / 1000;
        float timeDifference = (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000;
        lastTime = tap::arch::clock::getTimeMilliseconds();
        pid.runController(error, errorDerivative, timeDifference);
        motor.setDesiredOutput(pid.getOutput() + FEEDFORWARD);
    }
    else
    {
        motor.setDesiredOutput(motorDesiredOutput);
    }
}
}  // namespace aruwsrc::robot::engineer
