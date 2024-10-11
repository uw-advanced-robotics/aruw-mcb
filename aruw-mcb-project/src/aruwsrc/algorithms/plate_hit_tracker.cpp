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

namespace aruwsrc::algorithms
{
PlateHitTracker::PlateHitTracker(tap::Drivers *drivers)
    : drivers(drivers),
      transformer(nullptr),
      BLUR_CONVOLVE_MATRIX(BLUR_CONVOLVE_MATRIX_DATA)
{
}

void PlateHitTracker::initialize()
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
    if (transformer == nullptr)
    {
        return;
    }
    bins = bins * DECAY_FACTOR;
    if (this->drivers->refSerial.getRobotData().receivedDps > lastDPS)
    {
        hitTimer.restart(HIT_EXPIRE_TIME);
        dataTimestamp = this->drivers->refSerial.getRobotData().robotDataReceivedTimestamp;
        hitRecently = true;
        lastHitPlateID = static_cast<int>(this->drivers->refSerial.getRobotData().damagedArmorId);

        hitAngle_chassisRelative_radians = lastHitPlateID * M_PI / 2;
        hitAngle_worldRelative_radians =
            transformer->getWorldToChassis().getYaw() + hitAngle_chassisRelative_radians;

        // Update bins
        tap::algorithms::WrappedFloat selectedBin =
            tap::algorithms::WrappedFloat(hitAngle_worldRelative_radians, 0, 2 * M_PI);
        int binIndex = static_cast<int>(selectedBin.getWrappedValue() / (2 * M_PI / BIN_NUMBER));
        // Add the hit to the bin
        // Magnitude is based on damage
        bins.data[binIndex] += abs(this->drivers->refSerial.getRobotData().receivedDps - lastDPS);
    }
    lastDPS = this->drivers->refSerial.getRobotData().receivedDps;
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

tap::algorithms::CMSISMat<8, 1> PlateHitTracker::normaliseBins(tap::algorithms::CMSISMat<8, 1> mat)
{
    float sum = 0;
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        sum += mat.data[i];
    }
    if (sum == 0)
    {
        return mat;
    }
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        mat.data[i] = mat.data[i] / sum;
    }
    return mat;
}
tap::algorithms::CMSISMat<8, 1> PlateHitTracker::blurBins(tap::algorithms::CMSISMat<8, 1> mat)
{
    return BLUR_CONVOLVE_MATRIX * mat;
}

float PlateHitTracker::getPeakAngleDegrees()
{
    bins = normaliseBins(bins);
    tap::algorithms::CMSISMat<8, 1> temp = blurBins(bins);
    float max = 0;
    int maxIndex = 0;
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        if (temp.data[i] > max)
        {
            max = temp.data[i];
            maxIndex = i;
        }
    }

    int8_t prevBin = (maxIndex - 1 < 0) ? 7 : maxIndex - 1;
    int8_t nextBin = (maxIndex + 1 > 7) ? 0 : maxIndex + 1;

    float peakAngleRaw =
        maxIndex * 45 - (45 * prevBin * temp.data[prevBin]) + (45 * nextBin * temp.data[nextBin]);
    tap::algorithms::WrappedFloat peakAngleWrapped(peakAngleRaw, 0, 360);

    lastPeakAngleDegrees = peakAngleWrapped.getWrappedValue();
    return peakAngleWrapped.getWrappedValue();
}

std::array<PlateHitTracker::PlateHitBinData, 10> PlateHitTracker::getPeakData()
{
    std::array<PlateHitBinData, 10> peakData;
    for (int i = 0; i < 10; i++)
    {
        peakData[i].degrees = i * 45;
        peakData[i].magnitude = bins.data[i];
    }
    return peakData;
}

}  // namespace aruwsrc::algorithms