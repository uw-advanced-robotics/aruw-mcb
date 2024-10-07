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

#ifndef PLATE_HIT_TRACKER_HPP_
#define PLATE_HIT_TRACKER_HPP_

#include <tap/architecture/timeout.hpp>

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/drivers.hpp"

#include "modm/math/matrix.hpp"

namespace aruwsrc::communication::serial
{
class PlateHitTracker
{
    struct PlateHitData
    {
        int plateID;
        float hitAngleChassisRadians;
        float hitAngleWorldRadians;
        int timestamp;
    };

public:
    PlateHitTracker(tap::Drivers *drivers);
    PlateHitData getRecentHitData();
    PlateHitData getLastHitData();
    bool isHitRecently();
    void initalize();
    void update();
    float getPeakAngleDegrees();

private:
    tap::algorithms::CMSISMat<10, 1> normaliseBins(tap::algorithms::CMSISMat<10, 1> mat);
    tap::algorithms::CMSISMat<10, 1> blurBins(tap::algorithms::CMSISMat<10, 1> mat);
    tap::Drivers *drivers;
    int dataTimestamp;
    float hitAngle_chassisRelative_radians;
    float hitAngle_worldRelative_radians;
    bool hitRecently;
    float lastDPS;
    int lastHitPlateID;
    tap::arch::MilliTimeout hitTimer;
    const int HIT_EXPIRE_TIME = 1000;
    const uint8_t BIN_NUMBER = 10;
    tap::algorithms::CMSISMat<10, 1> bins;
    const float DECAY_FACTOR = 0.95;
    float lastPeakAngleDegrees;

    const tap::algorithms::CMSISMat<10, 10> BLUR_CONVOLVE_MATRIX;
    // clang-format off
    static constexpr float BLUR_CONVOLVE_MATRIX_DATA[100] = {
        0.5 , 0.25, 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0.25,
        0.25, 0.5 , 0.25, 0   , 0   , 0   , 0   , 0   , 0   , 0   ,
        0   , 0.25, 0.5 , 0.25, 0   , 0   , 0   , 0   , 0   , 0   ,
        0   , 0   , 0.25, 0.5 , 0.25, 0   , 0   , 0   , 0   , 0   ,
        0   , 0   , 0   , 0.25, 0.5 , 0.25, 0   , 0   , 0   , 0   ,
        0   , 0   , 0   , 0   , 0.25, 0.5 , 0.25, 0   , 0   , 0   ,
        0   , 0   , 0   , 0   , 0   , 0.25, 0.5 , 0.25, 0   , 0   ,
        0   , 0   , 0   , 0   , 0   , 0   , 0.25, 0.5 , 0.25, 0   ,
        0   , 0   , 0   , 0   , 0   , 0   , 0   , 0.25, 0.5 , 0.25,
        0.25, 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0.25, 0.5  
    };
    // clang-format on
};

}  // namespace aruwsrc::communication::serial
#endif  // PLATE_HIT_TRACKER_HPP_