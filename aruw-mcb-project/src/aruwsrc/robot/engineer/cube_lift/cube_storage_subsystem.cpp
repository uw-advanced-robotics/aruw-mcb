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
      motor(storageLiftMotor)
{
    calibrationState = CalibrationState::AWAITING_CALIBRATE;
};

void CubeStorageSubsystem::initialize()
{
    motor.initialize();
    setDesiredOutput(0);
    pidState = PIDState::NONE;
}

void CubeStorageSubsystem::setDesiredOutput(int16_t power)
{
    motorDesiredOutput = power + FEEDFORWARD;
}

bool CubeStorageSubsystem::homedAndBounded() const
{
    return calibrationState == CalibrationState::CALIBRATION_COMPLETE;
}

void CubeStorageSubsystem::stopDuringHoming()
{
    pidState = PIDState::NONE;
    setDesiredOutput(0);
}

void CubeStorageSubsystem::setPositionSetpoint(float newSetpoint) { setpoint = newSetpoint; }

float CubeStorageSubsystem::getPositionSetpoint() { return setpoint; }

void CubeStorageSubsystem::setVelocitySetpoint(float newSetpoint)
{
    velocitySetpoint = newSetpoint;
}

float CubeStorageSubsystem::getVelocitySetpoint() { return velocitySetpoint; }

void CubeStorageSubsystem::setPIDState(PIDState state) { pidState = state; }

PIDState CubeStorageSubsystem::getPIDState() { return pidState; }

void CubeStorageSubsystem::moveTowardLowerBound() { pidState = PIDState::VELOCITY_PID; }

void CubeStorageSubsystem::setHome(uint64_t encoderPosition) { home = encoderPosition; }

void CubeStorageSubsystem::setUpperBound(uint64_t encoderPosition) { upperBound = encoderPosition; }

uint64_t CubeStorageSubsystem::getUpperBound() const { return upperBound; }

void CubeStorageSubsystem::setLowerBound(uint64_t encoderPosition) { lowerBound = encoderPosition; }

uint64_t CubeStorageSubsystem::getLowerBound() const { return lowerBound; }

void CubeStorageSubsystem::refreshSafeDisconnect()
{
    pidState = PIDState::NONE;
    motor.setDesiredOutput(0);
    motorDesiredOutput = 0;
}

bool CubeStorageSubsystem::isLimitSwitched()
{
    return !drivers->digital.read(CUBELIFT_LIMITSWITCH_PORT);
}

void CubeStorageSubsystem::refresh()
{
    isLimitSwitch = isLimitSwitched();
    if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
    {
        setDesiredOutput(homingOutput);  // debugging only
        if (!trigger.isTriggered())
        {
            calibrationState = CalibrationState::CALIBRATION_COMPLETE;
            motor.getEncoder()->resetEncoderValue();
            pidState = PIDState::NONE;
            setDesiredOutput(0);
        }
        else
        {
            moveTowardLowerBound();
        }
    }

    if (pidState == PIDState::POSITION_PID &&
        calibrationState == CalibrationState::CALIBRATION_COMPLETE)
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
    else if (pidState == PIDState::VELOCITY_PID)
    {
        float error = velocitySetpoint -
                      motor.getEncoder()->getVelocity() / 1000 / 60 / MM_PER_REVOLUTION / 1000;
        float timeDifference = (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000;
        lastTime = tap::arch::clock::getTimeMilliseconds();
        float errorDerivative = homingPID.runControllerDerivateError(error, timeDifference);
        homingPID.runController(error, errorDerivative, timeDifference);
        motor.setDesiredOutput(pid.getOutput());
    }
    else
    {
        motor.setDesiredOutput(motorDesiredOutput);
    }
}
}  // namespace aruwsrc::robot::engineer
