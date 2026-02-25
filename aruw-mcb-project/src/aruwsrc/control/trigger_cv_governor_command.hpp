/*
* Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef TRIGGER_CV_GOVERNOR_COMMAND_HPP_
#define TRIGGER_CV_GOVERNOR_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"

namespace aruwsrc::control
{
class ToggleCvGovernorCommand : public tap::control::Command
{
public:
    ToggleCvGovernorCommand(governor::CvOnTargetGovernor *governor, bool initial)
        : governor(governor), enabled(initial) {}

    void initialize() override
    {
        enabled = !enabled;
        governor->setGovernorEnabled(enabled);
    }

    bool isReady() override { return true; }

    void execute() override {}

    void end(bool) override {}

    bool isFinished() const override { return true; }

    const char* getName() const override { return "toggle cv governor command"; }

private:
    governor::CvOnTargetGovernor *governor;
    bool enabled;
};
} // namespace aruwsrc::control
#endif  // TRIGGER_CV_GOVERNOR_COMMAND_HPP_
