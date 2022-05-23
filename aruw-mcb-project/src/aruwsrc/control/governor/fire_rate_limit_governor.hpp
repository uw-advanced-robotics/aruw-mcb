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

#ifndef FIRE_RATE_LIMIT_GOVERNOR_HPP_
#define FIRE_RATE_LIMIT_GOVERNOR_HPP_

#include "tap/control/governor/command_governor_interface.hpp"
#include "modm/processing/timer/periodic_timer.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "tap/architecture/periodic_timer.hpp"

namespace aruwsrc::control::governor
{
/**
 * A governor that allows a Command to run when a TurretCVCommand has acquired and is aiming at a
 * target.
 */
class CvHasTargetGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    CvHasTargetGovernor(aruwsrc::serial::VisionCoprocessor &visionCoprocessor, uint8_t turretID)
        : visionCoprocessor(visionCoprocessor),
          turretID(turretID),
    {
    }

    void initialize() final
    {
        restartTimer();
    }

    bool isReady() final
    {
        return isTimerFinished();
    }

    bool isFinished() final { return !isReady(); }

private:
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    uint8_t turretID;

    tap::arch::MilliTimeout timer;
    uint32_t firerateMs1 = 3;
    uint32_t firerateMs2 = 2;
    uint32_t firerateMs3 = 1;

    bool isTimerFinished() {
        if(visionCoprocessor.getLastAimData(turretID).firerate == 0) return false;
        return timer.isExpired();
    }

    void restartTimer() {
        switch(visionCoprocessor.getLastAimData(turretID).firerate) {
            case 0:
                break;
            case 1:
                timer.restart(firerateMs1);
                break;
            case 2:
                timer.restart(firerateMs2);
                break;
            case 3:
                timer.restart(firerateMs3);
                break;
            default:
                //TODO add error message here
        }
    }

};
}  // namespace aruwsrc::control::governor

#endif  // FIRE_RATE_LIMIT_GOVERNOR_HPP_