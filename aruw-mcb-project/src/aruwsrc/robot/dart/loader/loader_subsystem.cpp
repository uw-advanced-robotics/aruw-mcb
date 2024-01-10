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

#include "loader_subsystem.hpp"

namespace aruwsrc::robot::dart
{
LoaderSubsystem::LoaderSubsystem(
    tap::Drivers* drivers,
    tap::motor::DjiMotor* loaderTopMotor,
    tap::motor::DjiMotor* loaderMiddleMotor,
    tap::motor::DjiMotor* loaderBottomMotor,
    const tap::algorithms::SmoothPidConfig& pidParams)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      loaderTopMotor(loaderTopMotor),
      loaderMiddleMotor(loaderMiddleMotor),
      loaderBottomMotor(loaderBottomMotor),
      topMotorPID(pidParams),
      middleMotorPID(pidParams),
      bottomMotorPID(pidParams),
      pidParams(pidParams)
{
}

void LoaderSubsystem::initialize()
{
    loaderTopMotor->initialize();
    loaderMiddleMotor->initialize();
    loaderBottomMotor->initialize();
}

void LoaderSubsystem::refresh()
{
    if (usingPID)
    {
        const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
        const uint32_t dt = curTime - prevTime;
        prevTime = curTime;

        uint64_t topPos = loaderTopMotor->getEncoderUnwrapped();
        uint64_t topMotorError = topMotorSetpoint - topPos;
        topMotorPID.runControllerDerivateError(topMotorError, dt);
        float topMotorOutput = topMotorPID.getOutput();
        loaderTopMotor->setDesiredOutput(topMotorOutput);

        uint64_t middlePos = loaderMiddleMotor->getEncoderUnwrapped();
        uint64_t middleMotorError = middleMotorSetpoint - middlePos;
        middleMotorPID.runControllerDerivateError(middleMotorError, dt);
        float middleMotorOutput = middleMotorPID.getOutput();
        loaderMiddleMotor->setDesiredOutput(middleMotorOutput);

        uint64_t bottomPos = loaderBottomMotor->getEncoderUnwrapped();
        uint64_t bottomMotorError = bottomMotorSetpoint - bottomPos;
        bottomMotorPID.runControllerDerivateError(bottomMotorError, dt);
        float bottomMotorOutput = bottomMotorPID.getOutput();
        loaderBottomMotor->setDesiredOutput(bottomMotorOutput);
    }
}

void LoaderSubsystem::setSetpoint(
    uint64_t topMotorSetpoint,
    uint64_t middleMotorSetpoint,
    uint64_t bottomMotorSetpoint)
{
    this->topMotorSetpoint = topMotorSetpoint;
    this->middleMotorSetpoint = middleMotorSetpoint;
    this->bottomMotorSetpoint = bottomMotorSetpoint;
    usingPID = true;
}

void LoaderSubsystem::setMotors(
    int16_t topMotorSpeed,
    int16_t middleMotorSpeed,
    int16_t bottomMotorSpeed)
{
    loaderTopMotor->setDesiredOutput(topMotorSpeed);
    loaderMiddleMotor->setDesiredOutput(middleMotorSpeed);
    loaderBottomMotor->setDesiredOutput(bottomMotorSpeed);
    usingPID = false;
}

void LoaderSubsystem::stop() { setMotors(0, 0, 0); }

bool LoaderSubsystem::atSetpoint(LoadingMotor motor)
{
    switch (motor)
    {
        case TOP:
            return fabsl(loaderTopMotor->getEncoderUnwrapped() - topMotorSetpoint) <=
                   pidParams.errDeadzone;
        case MIDDLE:
            return fabsl(loaderMiddleMotor->getEncoderUnwrapped() - middleMotorSetpoint) <=
                   pidParams.errDeadzone;
        case BOTTOM:
            return fabsl(loaderBottomMotor->getEncoderUnwrapped() - bottomMotorSetpoint) <=
                   pidParams.errDeadzone;
    }
    return true;
}

}  // namespace aruwsrc::robot::dart
