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
#ifndef DIGITAL_OUT_COMMAND_HPP_
#define DIGITAL_OUT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "digital_out_subsystem.hpp"

namespace aruwsrc::control::digital
{
class DigitalOutCommand : public tap::control::Command
{
public:
    DigitalOutCommand(DigitalOutSubsystem& subsystem, const bool state)
        : subsystem(subsystem),
          state(state),
          running(false)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(&subsystem));
    }

    inline void initialize() override {}

    inline void execute() override
    {
        subsystem.set(state);
        running = true;
    }

    inline void end(bool) override
    {
        subsystem.refreshSafeDisconnect();
        running = false;
    }

    inline bool isFinished() const override { return false; }

    const char* getName() const override { return "Digital Output Command"; }

private:
    DigitalOutSubsystem& subsystem;
    const bool state;
    bool running;
};  // class DigitalOutCommand

}  // namespace aruwsrc::control::digital
#endif  // DIGITAL_OUT_COMMAND_HPP_
