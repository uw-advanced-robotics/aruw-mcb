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
#ifndef MOTOR_HOMING_COMMAND_HPP_
#define MOTOR_HOMING_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/homeable_subsystem_interface.hpp"

namespace aruwsrc::control
{
/**
 * A command whose job is to locate and set the upper and lower bounds of the motor in a homeable
 * subsystem. When this command is scheduled, it performs the following actions:
 * 1. Command the motor to move toward its lower bound.
 * 2. Constantly detect the motion of the motor and determine when it stalls. Then, set the lower
 * bound to its position at this point.
 * 3. Command the motor to move in the opposite direction.
 * 4. Constantly detect the motion of the motor and determine when it stalls. Then, set the lower
 * bound to its position at this point.
 * 5. At this point, the motor homing is complete. End the command.
 */
class MotorHomingCommand : public tap::control::Command
{
public:

    /**
     * Specifies the state that the homing command is in.
     */
    enum class HomingState
    {
        /** While in this state, the motor is commanded to move toward the lower bound. */
        INITIATE_MOVE_TOWARD_LOWER_BOUND,
        /** While in this state, the command waits for the motor to stall. It then sets the lower
        bound of the subsystem to the subsystem's current position. */
        MOVING_TOWARD_LOWER_BOUND,
        /** While in this state, the motor is commanded to move toward the upper bound. */
        INITIATE_MOVE_TOWARD_UPPER_BOUND,
        /** While in this state, the command waits for the motor to stall. It then sets the upper
        bound of the subsystem to the subsystem's current position. */
        MOVING_TOWARD_UPPER_BOUND,
        /** While in this state, the command is finished. */
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
#endif
