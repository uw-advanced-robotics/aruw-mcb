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

#ifndef LAUNCHER_PULL_SUBSYSTEM_HPP_
#define LAUNCHER_PULL_SUBSYSTEM_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/robot/dart/dart_constants.hpp"

namespace aruwsrc::robot::dart
{
/**
 * Subsystems whose primary purpose is to drive the pulling mechanism on the dart launcher.
 */
class LauncherPullSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param drivers Pointer to robot drivers
     * @param launcherPullMotor1 The first motor to drive
     * @param launcherPullMotor2 The second motor to drive
     * @param launcherDeadMotor The motor being used as encoder on the pulley final stage
     * @param pidParams The PID parameters for the drive motors
     */
    LauncherPullSubsystem(
        tap::Drivers* drivers,
        tap::motor::DjiMotor* launcherPullMotor1,
        tap::motor::DjiMotor* launcherPullMotor2,
        tap::motor::DjiMotor* launcherDeadMotor,
        const tap::algorithms::SmoothPidConfig& pidParams);
    
    int64_t deadMotorEncoderVal = 0;

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        launcherDeadMotor->setDesiredOutput(0);
        launcherPullMotor1->setDesiredOutput(0);
        launcherPullMotor2->setDesiredOutput(0);
        deadMotorEncoderVal = launcherDeadMotor->getEncoderUnwrapped();
    }

    /**
     * Manually controls the motor output and blocks PID control until a new setpoint is set
     */
    void setMotor(int32_t motorSpeed);

    /**
     * Sets the setpoint for the PID controller and reenables PID control
     */
    void setSetpoint(uint64_t setpoint);

    void stop();

    inline bool allMotorsOnline()
    {
        return launcherPullMotor1->isMotorOnline() && launcherPullMotor2->isMotorOnline() &&
               launcherDeadMotor->isMotorOnline();
    };

    inline bool atSetpoint()
    {
        return fabsl(setpoint - launcherDeadMotor->getEncoderUnwrapped()) < pidParams.errDeadzone;
    }

    const char* getName() override { return "Launcher Pull Subsystem"; };

private:
    tap::Drivers* drivers;

    tap::motor::DjiMotor* launcherPullMotor1;
    tap::motor::DjiMotor* launcherPullMotor2;
    tap::motor::DjiMotor* launcherDeadMotor;

    tap::algorithms::SmoothPid pid;
    const tap::algorithms::SmoothPidConfig& pidParams;

    uint32_t prevTime = 0;
    bool isUsingPID = false;
    uint64_t setpoint = 0;
};
}  // namespace aruwsrc::robot::dart

#endif