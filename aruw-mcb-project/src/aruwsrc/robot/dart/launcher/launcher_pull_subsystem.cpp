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

#include "launcher_pull_subsystem.hpp"

#include "tap/architecture/clock.hpp"

namespace aruwsrc::robot::dart
{
LauncherPullSubsystem::LauncherPullSubsystem(
    tap::Drivers* drivers,
    tap::motor::DjiMotor* launcherPullMotor1,
    tap::motor::DjiMotor* launcherPullMotor2,
    tap::motor::DjiMotor* launcherDeadMotor,
    const tap::algorithms::SmoothPidConfig& pidParams)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      launcherPullMotor1(launcherPullMotor1),
      launcherPullMotor2(launcherPullMotor2),
      launcherDeadMotor(launcherDeadMotor),
      pid(pidParams),
      pidParams(pidParams)
{
}

void LauncherPullSubsystem::refresh()
{
    if (isUsingPID)
    {
        const uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
        const uint32_t dt = curTime - prevTime;
        prevTime = curTime;

        uint64_t pos = launcherDeadMotor->getEncoderUnwrapped();
        uint64_t error = setpoint - pos;
        pid.runControllerDerivateError(error, dt);
        float output = pid.getOutput();

        launcherPullMotor1->setDesiredOutput(output);
        launcherPullMotor2->setDesiredOutput(output);
    }

    deadMotorEncoderVal = launcherDeadMotor->getEncoderUnwrapped();
}

void LauncherPullSubsystem::initialize()
{
    launcherPullMotor1->initialize();
    launcherPullMotor2->initialize();
    launcherDeadMotor->initialize();
}

void LauncherPullSubsystem::setMotor(int32_t motorSpeed)
{
    launcherPullMotor1->setDesiredOutput(motorSpeed);
    launcherPullMotor2->setDesiredOutput(motorSpeed);
    isUsingPID = false;
}

void LauncherPullSubsystem::stop() { setMotor(0); }

void LauncherPullSubsystem::setSetpoint(uint64_t setpoint)
{
    this->setpoint = setpoint;
    isUsingPID = true;
}

}  // namespace aruwsrc::robot::dart