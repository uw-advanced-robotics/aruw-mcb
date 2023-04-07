/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/homeable_subsystem_interface.hpp"

namespace aruwsrc::control
{
class MotorHomingCommand : public tap::control::Command
{
public:
    static constexpr int32_t HOMING_MOTOR_OUTPUT = SHRT_MAX / 2;

    enum class HomingState
    {
        INITIATE_MOVE_TOWARD_LOWER_BOUND,
        MOVING_TOWARD_LOWER_BOUND,
        INITIATE_MOVE_TOWARD_UPPER_BOUND,
        MOVING_TOWARD_UPPER_BOUND,
        HOMING_COMPLETE
    };

    MotorHomingCommand(
        aruwsrc::control::HomeableSubsystemInterface& subsystem,
        tap::Drivers& drivers)
        : subsystem(subsystem),
          drivers(drivers)
    {
        addSubsystemRequirement(&subsystem);
    };

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupt) override;

private:
    aruwsrc::control::HomeableSubsystemInterface& subsystem;
    tap::Drivers& drivers;
    HomingState homingState;
};  // class MotorHomingCommand
}  // namespace aruwsrc::control