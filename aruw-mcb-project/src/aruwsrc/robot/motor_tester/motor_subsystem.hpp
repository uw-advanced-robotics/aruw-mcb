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

#include <algorithm>

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"

namespace aruwsrc::motor_tester
{

class MotorSubsystem final : public tap::control::Subsystem
{
public:
    MotorSubsystem(tap::Drivers* drivers, tap::motor::MotorInterface& motor)
        : Subsystem(drivers),
          motor(motor)
    {
    }

    inline void initialize() override { motor.initialize(); };

    void setDesiredOutput(int32_t desiredOutput)
    {
        this->desiredOutput = std::clamp(
            desiredOutput,
            -static_cast<int32_t>(tap::motor::DjiMotor::MAX_OUTPUT_C620),
            static_cast<int32_t>(tap::motor::DjiMotor::MAX_OUTPUT_C620);)
    }

    int32_t getDesiredOutput() const { return desiredOutput; }

    inline void refresh() override { motor.setDesiredOutput(desiredOutput); };

private:
    tap::motor::MotorInterface motor;
    int32_t desiredOutput = 0;
}

}  // namespace aruwsrc::motor_tester

#endif  // MOTOR_SUBSYSTEM_HPP_