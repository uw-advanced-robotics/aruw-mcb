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
    motor.getEncoder()->resetEncoderValue();
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

int slow = 10;
void CubeStorageSubsystem::moveTowardLowerBound() { 
    pidState = PIDState::NONE; 
    // if(motor.getEncoder()->getVelocity() > slow) {
    //     motorDesiredOutput = homingOutput;
    // } else {
    //     motorDesiredOutput = homingOutput - ;
    // } //TODO: fix this
    motorDesiredOutput = homingOutput;
}

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

float motorPos = 0;
float pidOutput = 1;
float feedforward = 0;
void CubeStorageSubsystem::refresh()
{   
    isLimitSwitch = isLimitSwitched();
    caliState = calibrationState;
    
    if (calibrationState == CalibrationState::CALIBRATING_LOWER_BOUND)
    {
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

    if (pidState == PIDState::POSITION_PID)
    {
        motorPos =  motor.getEncoder()->getPosition().getUnwrappedValue()  / (tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 * tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508) / M_TWOPI * MM_PER_REVOLUTION;
        float error =
            setpoint - motorPos;
        float errorDerivative =
            motor.getEncoder()->getVelocity() / (tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508 * tap::motor::DjiMotorEncoder::GEAR_RATIO_M3508) / M_TWOPI * MM_PER_REVOLUTION;
        float timeDifference = (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000.0f; // (s)
        lastTime = tap::arch::clock::getTimeMilliseconds();
        pid.runController(error, errorDerivative, timeDifference);
        motor.setDesiredOutput(pid.getOutput() + FEEDFORWARD);
        pidOutput = pid.getOutput();
    }
    else if (pidState == PIDState::VELOCITY_PID)
    {
        float error = velocitySetpoint -
                      motor.getEncoder()->getVelocity() / 1000 / 60 / MM_PER_REVOLUTION / 1000;
        float timeDifference = (tap::arch::clock::getTimeMilliseconds() - lastTime) / 1000;
        lastTime = tap::arch::clock::getTimeMilliseconds();
        homingPID.runControllerDerivateError(error, timeDifference);
        motor.setDesiredOutput(homingPID.getOutput());
    }
    else
    {
        motor.setDesiredOutput(motorDesiredOutput);
    }
}
}  // namespace aruwsrc::robot::engineer
