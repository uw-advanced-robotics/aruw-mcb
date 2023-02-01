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

#ifndef CV_ONLINE_GOVERNOR_HPP_
#define CV_ONLINE_GOVERNOR_HPP_

#include "tap/control/governor/command_governor_interface.hpp"

#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::governor
{
/**
 * A governor that allows a Command to execute when the vision coprocessor is connected.
 */
class CvOnlineGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    CvOnlineGovernor(
        tap::Drivers &drivers,
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand)
        : drivers(drivers),
          visionCoprocessor(visionCoprocessor),
          turretCVCommand(turretCVCommand)
    {
    }

    bool isReady() final
    {
        return visionCoprocessor.isCvOnline() &&
               drivers.commandScheduler.isCommandScheduled(&turretCVCommand);
    }

    bool isFinished() final { return !isReady(); }

private:
    tap::Drivers &drivers;
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand;
};
}  // namespace aruwsrc::control::governor

#endif  // CV_ONLINE_GOVERNOR_HPP_
