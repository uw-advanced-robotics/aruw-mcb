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

#ifndef LOADER_SUBSYSTEM_HPP_
#define LOADER_SUBSYSTEM_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::robot::dart
{
enum LoadingMotor
{
    TOP,
    MIDDLE,
    BOTTOM
};

/**
 * Subsystem whose primary purpose is to drive the control of the dart loading mechanism.
 */
class LoaderSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param drivers Pointer to robot drivers
     * @param loaderTopMotor The top motor of the loader
     * @param loaderMiddleMotor The middle motor of the loader
     * @param loaderBottomMotor The bottom motor of the loader
     * @param pidParams The PID parameters for the motors
     */
    LoaderSubsystem(
        tap::Drivers* drivers,
        tap::motor::DjiMotor* loaderTopMotor,
        tap::motor::DjiMotor* loaderMiddleMotor,
        tap::motor::DjiMotor* loaderBottomMotor,
        const tap::algorithms::SmoothPidConfig& pidParams);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        loaderTopMotor->setDesiredOutput(0);
        loaderMiddleMotor->setDesiredOutput(0);
        loaderBottomMotor->setDesiredOutput(0);
    }

    /**
     * Sets PID setpoint for all motors and enables PID control
     */
    void setSetpoint(
        uint64_t topMotorSetpoint,
        uint64_t middleMotorSetpoint,
        uint64_t bottomMotorSetpoint);

    /**
     * Manually sets all motors and disables PID control
     */
    void setMotors(int16_t topMotorSpeed, int16_t middleMotorSpeed, int16_t bottomMotorSpeed);

    void stop();

    const char* getName() override { return "Pivot Subsystem"; };

    inline bool allMotorsOnline()
    {
        topMotorOnline = loaderTopMotor->isMotorOnline();
        middleMotorOnline = loaderMiddleMotor->isMotorOnline();
        bottomMotorOnline = loaderBottomMotor->isMotorOnline();

        return loaderTopMotor->isMotorOnline() && loaderMiddleMotor->isMotorOnline() &&
               loaderBottomMotor->isMotorOnline();
    }

    bool atSetpoint(LoadingMotor motor);

    bool topMotorOnline = false, middleMotorOnline = false, bottomMotorOnline = false;

private:
    tap::Drivers* drivers;
    tap::motor::DjiMotor* loaderTopMotor;
    tap::motor::DjiMotor* loaderMiddleMotor;
    tap::motor::DjiMotor* loaderBottomMotor;

    tap::algorithms::SmoothPid topMotorPID;
    tap::algorithms::SmoothPid middleMotorPID;
    tap::algorithms::SmoothPid bottomMotorPID;

    const tap::algorithms::SmoothPidConfig& pidParams;

    uint64_t topMotorSetpoint;
    uint64_t middleMotorSetpoint;
    uint64_t bottomMotorSetpoint;

    bool usingPID = false;
    uint32_t prevTime = 0;
};
}  // namespace aruwsrc::robot::dart

#endif
