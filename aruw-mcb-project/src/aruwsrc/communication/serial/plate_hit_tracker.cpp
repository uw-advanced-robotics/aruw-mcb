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
PlateHitTracker::PlateHitTracker(tap::Drivers *drivers)
    : drivers(drivers),
      BLUR_CONVOLVE_MATRIX(BLUR_CONVOLVE_MATRIX_DATA)
{
}

void PlateHitTracker::initalize()
{
    dataTimestamp = -1;
    hitAngle_chassisRelative_radians = -1;
    hitAngle_worldRelative_radians = -1;
    hitRecently = false;
    hitTimer.stop();
}

/**
 * @brief Updates the plate hit tracker with the latest data from the robot. Should be called in the
 * main loop.
 */
void PlateHitTracker::update()
{
    bins = bins * DECAY_FACTOR;
    if (this->drivers->refSerial.getRobotData().receivedDps > lastDPS)
    {
        hitTimer.restart(HIT_EXPIRE_TIME);
        dataTimestamp = this->drivers->refSerial.getRobotData().robotDataReceivedTimestamp;
        hitRecently = true;
        lastDPS = this->drivers->refSerial.getRobotData().receivedDps;
        lastHitPlateID = static_cast<int>(this->drivers->refSerial.getRobotData().damagedArmorId);

        hitAngle_chassisRelative_radians = lastHitPlateID * M_PI / 2;
        hitAngle_worldRelative_radians =
            (lastHitPlateID * M_PI / 2) + modm::toRadian(this->drivers->mpu6500.getYaw());

        // Update bins
        bins.data[modm::toDegree(hitAngle_worldRelative_radians) / (360 / BIN_NUMBER)] += 1;
    }
    hitRecently = !hitTimer.isExpired();
}

bool PlateHitTracker::isHitRecently() { return hitRecently; }

PlateHitTracker::PlateHitData PlateHitTracker::getLastHitData()
{
    PlateHitData hitData;
    hitData.plateID = lastHitPlateID;
    hitData.hitAngleChassisRadians = hitAngle_chassisRelative_radians;
    hitData.hitAngleWorldRadians = hitAngle_worldRelative_radians;
    hitData.timestamp = dataTimestamp;
    return hitData;
}

tap::algorithms::CMSISMat<10, 1> PlateHitTracker::normaliseBins(
    tap::algorithms::CMSISMat<10, 1> mat)
{
    float sum = 0;
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        sum += mat.data[i];
    }
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        mat.data[i] = mat.data[i] / sum;
    }
    return mat;
}
tap::algorithms::CMSISMat<10, 1> PlateHitTracker::blurBins(tap::algorithms::CMSISMat<10, 1> mat)
{
    return BLUR_CONVOLVE_MATRIX * mat;
}

float PlateHitTracker::getPeakAngleDegrees()
{
    bins = normaliseBins(bins);
    bins = blurBins(bins);
    float max = 0;
    int maxIndex = 0;
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        if (bins.data[i] > max)
        {
            max = bins.data[i];
            maxIndex = i;
        }
    }

    lastPeakAngleDegrees = ((((360 / BIN_NUMBER) * 2)) + (maxIndex * (360 / BIN_NUMBER))) +
                           ((((360 / BIN_NUMBER) * 2) + ((maxIndex + 1) * (360 / BIN_NUMBER))) *
                            (bins.data[maxIndex + 1])) -
                           ((((360 / BIN_NUMBER) * 2) + ((maxIndex - 1) * (360 / BIN_NUMBER))) *
                            (bins.data[maxIndex - 1]));

    return lastPeakAngleDegrees;
}

}  // namespace aruwsrc::communication::serial