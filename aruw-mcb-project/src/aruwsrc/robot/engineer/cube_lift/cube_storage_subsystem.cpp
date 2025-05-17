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
    setDesiredOutput(-1 * FEEDFORWARD);
}

// void CubeStorageSubsystem::setVelocitySetpoint(float newSetpoint)
// {
//     velocitySetpoint = newSetpoint;
// }

// float CubeStorageSubsystem::getVelocitySetpoint() { return velocitySetpoint; }
/*potentially use if whicher controls lead/aiden wants us to (we forgot which)*/

void CubeStorageSubsystem::moveTowardLowerBound()
{
    pidState = PIDState::POSITION_PID;
    setPositionSetpoint(HOMING_SPEED + getMotorPosition());
}

void CubeStorageSubsystem::setHome(uint64_t encoderPosition) { home = encoderPosition; }

void CubeStorageSubsystem::refreshSafeDisconnect()
{
    pidState = PIDState::NONE;
    motor.setDesiredOutput(0);
    motorDesiredOutput = 0;
}

float CubeStorageSubsystem::getMotorPosition()
{
    return motor.getEncoder()->getPosition().getUnwrappedValue() / M_TWOPI * MM_PER_REVOLUTION;
}

void CubeStorageSubsystem::refresh()
{
    if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
    {
        if (trigger.isTriggered())
        {
            calibrationState = CalibrationState::CALIBRATION_COMPLETE;
            motor.getEncoder()->resetEncoderValue();
            pidState = PIDState::POSITION_PID;
            setPositionSetpoint(ONE_CUBE_SETPOINT);
        }
        else
        {
            moveTowardLowerBound();
        }
    }

    if (pidState == PIDState::POSITION_PID)
    {
        motorPos =
            motor.getEncoder()->getPosition().getUnwrappedValue() / M_TWOPI * MM_PER_REVOLUTION;
        float error = setpoint - motorPos;
        float errorDerivative = motor.getEncoder()->getVelocity() / M_TWOPI * MM_PER_REVOLUTION;
        float timeDifference =
            (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000.0f;  // (s)
        lastTime = tap::arch::clock::getTimeMilliseconds();
        pid.runController(error, errorDerivative, timeDifference);
        motor.setDesiredOutput(pid.getOutput() + FEEDFORWARD);
        pidOutput = pid.getOutput();
    }
    else if (pidState == PIDState::VELOCITY_PID)
    {
        // float error = velocitySetpoint -
        //               motor.getEncoder()->getVelocity() / 1000 / 60 / MM_PER_REVOLUTION / 1000;
        // float timeDifference = (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000;
        // lastTime = tap::arch::clock::getTimeMilliseconds();
        // homingPID.runControllerDerivateError(error, timeDifference);
        // motor.setDesiredOutput(homingPID.getOutput());
        // TODO: fix math if we actually want to use
    }
    else
    {
        motor.setDesiredOutput(motorDesiredOutput);
    }
}
}  // namespace aruwsrc::robot::engineer
