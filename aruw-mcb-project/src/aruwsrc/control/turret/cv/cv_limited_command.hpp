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

#ifndef CV_LIMITED_COMMAND_HPP_
#define CV_LIMITED_COMMAND_HPP_

#include <vector>

#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret::cv
{
/**
 * A command that runs a command but only when CV has a target.
 */
class CVLimitedCommand : public tap::control::Command
{
public:
    /**
     * Constructs a CVLimitedCommand
     *
     * @param[in] drivers A reference to a global drivers object.
     * @param[in] subsystems The subsystem requirements of the command specified below. Mustn't be
     * any nullptr subsystems in the list.
     * @param[in] command A command that runs when CV is aiming at the target.
     * @param[in] turretCVCommand A turret CV command that the command uses to check if the turret
     * is aiming at the target.
     */
    CVLimitedCommand(
        aruwsrc::Drivers &drivers,
        const std::vector<tap::control::Subsystem *> &subsystemRequirements,
        tap::control::Command &command,
        const TurretCVCommand &turretCVCommand);

    const char *getName() const override { return "move CV limited"; }

    bool isReady() override;

    void initialize() override { return command.initialize(); }

    void execute() override { command.execute(); }

    void end(bool interrupted) override { command.end(interrupted); }

    bool isFinished() const override;

private:
    aruwsrc::Drivers &drivers;

    tap::control::Command &command;

    const aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand;

    /**
     * @return True if CV command running and the turret is aiming at the target.
     */
    bool visionAimingOnTarget() const;
};

}  // namespace aruwsrc::control::turret::cv

#endif  // CV_LIMITED_COMMAND_HPP_
