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
PlateHitTracker::PlateHitTracker(tap::Drivers* drivers)
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
        bins = normaliseBins(bins);
    }
    lastDPS = this->drivers->refSerial.getRobotData().receivedDps;
}

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

float PlateHitTracker::getPeakAngleRadians()
{
    auto peakData = getBinData();
    float max = 0;
    int maxIndex = 0;
    for (int i = 0; i < 8; i++)
    {
        if (peakData[i].magnitude > max)
        {
            max = peakData[i].magnitude;
            maxIndex = i;
        }
    }

    int8_t prevBin = (maxIndex - 1 < 0) ? 9 : maxIndex - 1;
    int8_t nextBin = (maxIndex + 1 > 9) ? 0 : maxIndex + 1;

    float peakAngleRaw = peakData[maxIndex].radians -
                         (peakData[prevBin].radians * peakData[prevBin].magnitude) +
                         (peakData[nextBin].radians * peakData[nextBin].magnitude);
    tap::algorithms::WrappedFloat peakAngleWrapped(peakAngleRaw, 0, 2 * M_PI);

    lastPeakAngleDegrees = peakAngleWrapped.getWrappedValue();
    return peakAngleWrapped.getWrappedValue();
}

PlateHitTracker::PlateHitBinData* PlateHitTracker::getBinData()
{
    static PlateHitBinData peakData[BIN_NUMBER];
    tap::algorithms::CMSISMat<8, 1> temp = blurBins(bins);
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        peakData[i].radians = modm::toRadian(i * 45);
        peakData[i].magnitude = temp.data[i];
    }
    return peakData;
}

}  // namespace aruwsrc::algorithms