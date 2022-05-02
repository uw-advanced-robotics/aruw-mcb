/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MOVE_CV_LIMITED_COMMAND_HPP_
#define MOVE_CV_LIMITED_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
/**
 */
class MoveCVLimitedCommand : public tap::control::Command
{
public:
    MoveCVLimitedCommand(
        aruwsrc::Drivers &drivers,
        tap::control::Subsystem &agitator,
        tap::control::Command &moveCommand,
        const aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand);

    const char *getName() const override { return "move CV limited"; }

    bool isReady() override;

    void initialize() override { return moveCommand.initialize(); }

    void execute() override { moveCommand.execute(); }

    void end(bool interrupted) override { moveCommand.end(interrupted); }

    bool isFinished() const override;

private:
    aruwsrc::Drivers &drivers;

    tap::control::Command &moveCommand;

    const aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand;

    bool visionAimingOnTarget() const;
};

}  // namespace aruwsrc::agitator

#endif  // MOVE_CV_LIMITED_COMMAND_HPP_
