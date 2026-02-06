/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

// add comments for why we made this for future ppls reference & why it isnt opposite
#ifndef DIGITAL_OUT_TOGGLE_COMMAND_HPP_
#define DIGITAL_OUT_TOGGLE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/robot/engineer/digital_out_subsystem.hpp"

namespace aruwsrc::engineer
{
class DigitalOutToggleCommand : public tap::control::Command
{
public:
    DigitalOutToggleCommand(DigitalOutSubsystem& subsystem)
        : subsystem(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(&subsystem));
    }

    inline void initialize() override
    {
        subsystem.set(!subsystem.getState());
    }

    inline void execute() override {}

    inline void end(bool) override {}

    inline bool isFinished() const override { return true; }

    const char* getName() const override { return "Digital Output Toggle Command"; }

private:
    DigitalOutSubsystem& subsystem;
};  // class DigitalOutToggleCommand

}  // namespace aruwsrc::engineer
#endif  // DIGITAL_OUT_TOGGLE_COMMAND_HPP_
