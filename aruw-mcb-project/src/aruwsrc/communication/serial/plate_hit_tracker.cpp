/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "plate_hit_tracker.hpp"

namespace aruwsrc::communication::serial
{
PlateHitTracker::PlateHitTracker(tap::Drivers *drivers) : drivers(drivers) {}

void PlateHitTracker::initalize()
{
    dataTimestamp = -1;
    hitAngle_chassisRelative_radians = -1;
    hitAngle_worldRelative_radians = -1;
    hitRecently = false;
    hitTimer.stop();
}

void PlateHitTracker::update()
{
    if (this->drivers->refSerial.getRobotData().receivedDps > lastDPS)
    {
        hitTimer.restart(HIT_EXPIRE_TIME);
        dataTimestamp = this->drivers->refSerial.getRobotData().robotDataReceivedTimestamp;
        hitRecently = true;
        lastDPS = this->drivers->refSerial.getRobotData().receivedDps;
        lastHitPlateID = static_cast<int>(this->drivers->refSerial.getRobotData().damagedArmorId);
    }
    hitRecently = !hitTimer.isExpired();
}

bool PlateHitTracker::isHitRecently() { return hitRecently; }

PlateHitData PlateHitTracker::getLastHitData()
{
    PlateHitData hitData;
    hitData.plateID = lastHitPlateID;
    hitData.hitAngleChassisRadians = lastHitPlateID * M_PI / 2;
    hitData.hitAngleWorldRadians =
        (lastHitPlateID * M_PI / 2) + modm::toRadian(this->drivers->mpu6500.getYaw());
    hitData.timestamp = dataTimestamp;
    return hitData;
}

}  // namespace aruwsrc::communication::serial