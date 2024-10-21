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

void PlateHitTracker::initialize() {}

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
    auto newHitData = this->drivers->refSerial.getRobotData();
    if (newHitData.receivedDps > lastHitData.lastDPS)
    {
        lastHitData.timestamp = newHitData.robotDataReceivedTimestamp;

        lastHitData.plateID = static_cast<int>(newHitData.damagedArmorId);

        lastHitData.hitAngle_chassisRelative_radians = Angle(lastHitData.plateID * M_PI / 2);
        lastHitData.hitAngle_worldRelative_radians = Angle(
            transformer->getWorldToChassis().getYaw() +
            lastHitData.hitAngle_chassisRelative_radians.getWrappedValue());

        // Update bins
        int binIndex = static_cast<int>(
            lastHitData.hitAngle_worldRelative_radians.getWrappedValue() / (2 * M_PI / BIN_NUMBER));
        // Add the hit to the bin
        // Magnitude is based on damage
        bins.data[binIndex] += newHitData.receivedDps - lastHitData.lastDPS;
    }
    lastHitData.lastDPS = newHitData.receivedDps;
}

CMSISMat<8, 1> PlateHitTracker::normaliseBins(CMSISMat<8, 1> mat)
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
CMSISMat<8, 1> PlateHitTracker::blurBins(CMSISMat<8, 1> mat) { return BLUR_CONVOLVE_MATRIX * mat; }

std::vector<PlateHitTracker::PlateHitBinData> PlateHitTracker::getPeakAnglesRadians()
{
    auto peakData = getBinData();
    std::vector<PlateHitBinData> peaks;

    for (int i = 0; i < BIN_NUMBER; i++)
    {
        int prevIndex = (i - 1 + BIN_NUMBER) % BIN_NUMBER;
        int nextIndex = (i + 1) % BIN_NUMBER;

        if (peakData[i].magnitude > peakData[prevIndex].magnitude &&
            peakData[i].magnitude > peakData[nextIndex].magnitude)
        {
            Angle peakAngleRaw = Angle(
                peakData[i].radians.getWrappedValue() -
                (peakData[prevIndex].radians.getWrappedValue() * peakData[prevIndex].magnitude) +
                (peakData[nextIndex].radians.getWrappedValue() * peakData[nextIndex].magnitude));
            peakData[i].radians = peakAngleRaw;
            peaks.push_back(peakData[i]);
        }
    }
    // sort by magnitude, so that index 0 is the biggest peak (where we are getting damaged the
    // most)
    std::sort(peaks.begin(), peaks.end(), [](const PlateHitBinData& a, const PlateHitBinData& b) {
        return a.magnitude > b.magnitude;
    });
    return peaks;
}

PlateHitTracker::PlateHitBinData* PlateHitTracker::getBinData()
{
    static PlateHitBinData peakData[BIN_NUMBER];

    CMSISMat<BIN_NUMBER, 1> temp = normaliseBins(bins);
    temp = blurBins(temp);
    for (int i = 0; i < BIN_NUMBER; i++)
    {
        peakData[i].radians = modm::toRadian(i * 45);
        peakData[i].magnitude = temp.data[i];
    }
    return peakData;
}

}  // namespace aruwsrc::algorithms