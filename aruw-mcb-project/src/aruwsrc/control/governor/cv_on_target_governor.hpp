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

#ifndef CV_ON_TARGET_GOVERNOR_HPP_
#define CV_ON_TARGET_GOVERNOR_HPP_

#include "tap/control/command_governor_interface.hpp"

#include "aruwsrc/control/turret/cv/turret_cv_command.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::governor
{
class CvOnTargetGovernor : public tap::control::CommandGovernorInterface
{
public:
    CvOnTargetGovernor(
        aruwsrc::Drivers &drivers,
        aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand)
        : drivers(drivers),
          turretCVCommand(turretCVCommand)
    {
    }

    bool isReady() final
    {
        return drivers.commandScheduler.isCommandScheduled(&turretCVCommand) &&
               turretCVCommand.isAimingWithinLaunchingTolerance();
    }

    bool isFinished() final { return !isReady(); }

private:
    aruwsrc::Drivers &drivers;
    aruwsrc::control::turret::cv::TurretCVCommand &turretCVCommand;
};
}  // namespace aruwsrc::agitator

#endif  // CV_ON_TARGET_GOVERNOR_HPP_