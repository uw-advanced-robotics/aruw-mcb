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

#include "tap/drivers.hpp"

#include <tap/architecture/timeout.hpp>

namespace aruwsrc::communication::serial
{
    class PlateHitTracker {
        public:
            PlateHitTracker(tap::Drivers  *drivers);
            PlateHitData getRecentHitData();
            PlateHitData getLastHitData();
            bool isHitRecently();
        private:
            void initalize();
            void update();
            tap::Drivers *drivers;
            int dataTimestamp;
            float hitAngle_chassisRelative_radians;
            float hitAngle_worldRelative_radians;
            bool hitRecently;
            float lastDPS;
            int lastHitPlateID;
            tap::arch::MilliTimeout hitTimer;
            const int HIT_EXPIRE_TIME = 1000;
    };

    struct PlateHitData
    {
        int plateID;
        float hitAngleChassisRadians;
        float hitAngleWorldRadians;
        int timestamp;
    };

}