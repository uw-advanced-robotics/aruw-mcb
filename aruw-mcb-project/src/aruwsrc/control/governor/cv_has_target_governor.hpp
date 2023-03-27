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

#ifndef CV_HAS_TARGET_GOVERNOR_HPP_
#define CV_HAS_TARGET_GOVERNOR_HPP_

#include "tap/control/governor/command_governor_interface.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc::control::governor
{
/**
 * A governor that allows a Command to execute when the vision coprocessor is connected and some
 * specified turret has a target.
 *
 * This governor serves a different purpose than the `CvOnTargetGovernor`. This governor purely
 * checks if vision has a target, whereas the `CvOnTargetGovernor` governor checks if a projectile
 * launched at the acquired target will hit it.
 */
class CvHasTargetGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    CvHasTargetGovernor(aruwsrc::serial::VisionCoprocessor &visionCoprocessor, uint8_t turretID)
        : visionCoprocessor(visionCoprocessor),
          turretID(turretID)
    {
    }

    bool isReady() final
    {
        return visionCoprocessor.isCvOnline() &&
               visionCoprocessor.getLastAimData(turretID).timing.updated;
    }

    bool isFinished() final { return !isReady(); }

private:
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    uint8_t turretID;
};
}  // namespace aruwsrc::control::governor

#endif  // CV_HAS_TARGET_GOVERNOR_HPP_
