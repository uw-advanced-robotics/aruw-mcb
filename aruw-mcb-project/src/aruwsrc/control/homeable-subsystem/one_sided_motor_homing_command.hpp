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
#ifndef ONE_SIDED_MOTOR_HOMING_COMMAND_HPP_
#define ONE_SIDED_MOTOR_HOMING_COMMAND_HPP_

#include "aruwsrc/control/homeable-subsystem/homeable_subsystem_interface.hpp"
#include "aruwsrc/control/homeable-subsystem/motor_homing_command_interface.hpp"
#include "aruwsrc/control/homeable-subsystem/trigger/trigger_interface.hpp"

namespace aruwsrc::control
{
class OneSidedMotorHomingCommand : public MotorHomingCommandInterface
{
public:
    OneSidedMotorHomingCommand(HomeableSubsystemInterface& subsystem, TriggerInterface& trigger)
        : MotorHomingCommandInterface(subsystem),
          trigger(trigger)
    {
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "One-Sided Motor homing"; }

private:
    TriggerInterface& trigger;
};
}  // namespace aruwsrc::control

#endif
