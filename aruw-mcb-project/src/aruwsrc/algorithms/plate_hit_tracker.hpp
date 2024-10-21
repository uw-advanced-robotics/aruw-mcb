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

#include <aruwsrc/algorithms/odometry/transformer_interface.hpp>
#include <tap/algorithms/wrapped_float.hpp>

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "modm/math/matrix.hpp"

using tap::algorithms::Angle;
using tap::algorithms::CMSISMat;
using tap::algorithms::WrappedFloat;

namespace aruwsrc::algorithms
{
class PlateHitTracker
{
    struct PlateHitData
    {
        int plateID;
        float lastDPS;
        float hitAngle_chassisRelative_radians;
        float hitAngle_worldRelative_radians;
        uint32_t timestamp;
        PlateHitData()
            : plateID(-1),
              lastDPS(-1),
              hitAngle_chassisRelative_radians(0),
              hitAngle_worldRelative_radians(0),
              timestamp(-1)
        {
        }
    };

    struct PlateHitBinData
    {
        Angle radians;
        float magnitude;
        PlateHitBinData() : radians(0), magnitude(0) {}
    };

public:
    /**
     * @brief PlateHitTracker
     * @param drivers Robot drivers
     * @param transforms TransformerInterface for getting the robot's orientation
     * @brief Checks refSerial for hit data and adds them to a matrix of bins
     * each bin in 45 degrees around the robot
     * bin 0 => 337.5 to 22.5 \n
     * bin 1 => 22.5 to 67.5\n
     * bin 2 => 67.5 to 112.5\n
     * bin 3 => 112.5 to 157.5\n
     * bin 4 => 157.5 to 202.5\n
     * bin 5 => 202.5 to 247.5\n
     * bin 6 => 247.5 to 292.5\n
     * bin 7 => 292.5 to 337.5\n
     * Final product is a list of peaks and their positions around the robot
     */
    PlateHitTracker(tap::Drivers *drivers);

    mockable inline PlateHitData getLastHitData() { return lastHitData; }

    std::vector<PlateHitBinData> getPeakAnglesRadians();

    void initialize();

    void update();

    mockable inline void attachTransformer(
        aruwsrc::algorithms::transforms::TransformerInterface *transformer)
    {
        this->transformer = transformer;
    }
    static constexpr uint8_t BIN_NUMBER = 8;

private:
    static constexpr float BLUR_FACTOR = 0.5;

    // clang-format off
        static constexpr float A = 0.5; 
        static constexpr float B = (1 - A) / 2; // Derived from 2B + A = 1
        static constexpr float BLUR_CONVOLVE_MATRIX_DATA[BIN_NUMBER*BIN_NUMBER] = {
            A , B, 0 , 0 , 0 , 0 , 0 , B,
            B , A , B, 0 , 0 , 0 , 0 , 0,
            0 , B , A , B, 0 , 0 , 0 , 0,
            0 , 0 , B , A , B, 0 , 0 , 0,
            0 , 0 , 0 , B , A , B, 0 , 0,
            0 , 0 , 0 , 0 , B , A , B, 0,
            0 , 0 , 0 , 0 , 0 , B , A , B,
            B , 0 , 0 , 0 , 0 , 0 , B , A
        };
    // clang-format on
    const float DECAY_FACTOR = 0.99995;

    // Variables
    tap::Drivers *drivers;
    PlateHitData lastHitData;

    aruwsrc::algorithms::transforms::TransformerInterface *transformer;
    CMSISMat<BIN_NUMBER, 1> bins;
    const CMSISMat<BIN_NUMBER, BIN_NUMBER> BLUR_CONVOLVE_MATRIX;

    PlateHitTracker::PlateHitBinData *getBinData();
    CMSISMat<BIN_NUMBER, 1> normaliseBins(CMSISMat<BIN_NUMBER, 1> mat);
    CMSISMat<BIN_NUMBER, 1> blurBins(CMSISMat<BIN_NUMBER, 1> mat);
};

}  // namespace aruwsrc::algorithms
#endif  // PLATE_HIT_TRACKER_HPP_