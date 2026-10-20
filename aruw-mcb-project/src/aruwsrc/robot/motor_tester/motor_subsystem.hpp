/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/motor_interface.hpp"

namespace aruwsrc::motor_tester
{

class MotorSubsystem final : public tap::control::Subsystem
{
public:
    MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        const tap::algorithms::SmoothPidConfig& pidConfig)
        : Subsystem(drivers),
          motor(motor),
          pidController(pidConfig)
    {
    }

    void initialize() override
    {
        motor.initialize();
        desiredPosition = motor.getEncoder()->getPosition().getUnwrappedValue();
    }

    void setDesiredPosition(float desiredPosition) { this->desiredPosition = desiredPosition; }

    float getDesiredPosition() const { return desiredPosition; }

    void refresh() override
    {
        const float output = pidController.runController(
            desiredPosition - motor.getEncoder()->getPosition().getUnwrappedValue(),
            -motor.getEncoder()->getVelocity(),
            0.002);
        motor.setDesiredOutput(static_cast<int32_t>(output));
    }

    void refreshSafeDisconnect() override { motor.setDesiredOutput(0); }

private:
    tap::motor::MotorInterface& motor;
    float desiredPosition = 0.0f;
    tap::algorithms::SmoothPid pidController;
};

}  // namespace aruwsrc::motor_tester

#endif  // MOTOR_SUBSYSTEM_HPP_
