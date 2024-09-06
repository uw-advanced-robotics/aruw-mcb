/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SUCK_IT_COMMAND_HPP_
#define SUCK_IT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "grabber_subsystem.hpp"

namespace aruwsrc::engineer::grabber
{

class SuckItCommand : public tap::control::Command
{
public:
    SuckItCommand(GrabberSubsystem* grabber) : grabber(grabber)
    {
        addSubsystemRequirement(grabber);
    }

    void initialize() override { grabber->toggle(); }

    void execute() override {}

    void end(bool interrupted) override {}

    bool isFinished() const override { return true; }

    const char* getName() const override { return "Suck it"; }

private:
    GrabberSubsystem* grabber;
};  // class SuckItCommand

}  // namespace aruwsrc::engineer::grabber
#endif  // SUCK_IT_COMMAND_HPP_
