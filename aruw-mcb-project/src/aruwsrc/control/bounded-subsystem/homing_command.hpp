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

#ifndef HOMING_COMMAND_HPP_
#define HOMING_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/bounded-subsystem/bounded_subsystem_interface.hpp"

namespace aruwsrc::control
{
/**
 * A command that tells a bounded subsystem to calibrate.
 */
class HomingCommand : public tap::control::Command
{
public:
    HomingCommand(aruwsrc::control::BoundedSubsystemInterface& subsystem) : subsystem(subsystem)
    {
        addSubsystemRequirement(&subsystem);
    }

    void initialize() override;

    bool isFinished() const override;

protected:
    aruwsrc::control::BoundedSubsystemInterface& subsystem;
};  // class HomingCommand
}  // namespace aruwsrc::control

#endif  // HOMING_COMMAND_HPP_
