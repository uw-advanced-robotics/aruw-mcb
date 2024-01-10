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

#ifndef PIVOT_SUBSYSTEM_HPP_
#define PIVOT_SUBSYSTEM_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::robot::dart
{
/**
 * Subsystems whose primary use to control the pivot motor of the dart launcher.
 */
class PivotSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param drivers The drivers object.
     * @param pivotMotor The pivot motor.
     * @param pivotDeadMotor The motor on the pivot axis being used for its encoder
     */
    PivotSubsystem(
        tap::Drivers* drivers,
        tap::motor::DjiMotor* pivotMotor,
        tap::motor::DjiMotor* pivotDeadMotor,
        const tap::algorithms::SmoothPidConfig& pidParams);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        pivotMotor->setDesiredOutput(0);
        pivotDeadMotor->setDesiredOutput(0);
    }

    void setSetpoint(uint64_t setpoint);

    void setMotor(int32_t motorSpeed);

    void stop();

    inline bool allMotorsOnline()
    {
        return pivotMotor->isMotorOnline() && pivotDeadMotor->isMotorOnline();
    }

    inline bool atSetpoint()
    {
        return fabsl(setpoint - pivotDeadMotor->getEncoderUnwrapped()) <= pidParams.errDeadzone;
    }

    const char* getName() override { return "Pivot Subsystem"; };

    static constexpr uint16_t TURN_SPEED = 1000;

private:
    tap::Drivers* drivers;

    tap::motor::DjiMotor* pivotMotor;
    tap::motor::DjiMotor* pivotDeadMotor;

    tap::algorithms::SmoothPid pid;
    const tap::algorithms::SmoothPidConfig& pidParams;

    uint32_t prevTime = 0;
    bool isUsingPID = false;
    uint64_t setpoint = 0;
};
}  // namespace aruwsrc::robot::dart

#endif