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
#include <tap/algorithms/wrapped_float.hpp>
#include "modm/math/matrix.hpp"
#include <aruwsrc/algorithms/odometry/transformer_interface.hpp>
#include "tap/control/subsystem.hpp"
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

    struct PlateHitBinData
    {
        float degrees;
        float magnitude;
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
    PlateHitTracker(
        tap::Drivers *drivers);
    PlateHitData getRecentHitData();
    PlateHitData getLastHitData();
    bool isHitRecently();
    float getPeakAngleDegrees();

    void initialize();

    void update();

    mockable inline void attachTransformer(
    aruwsrc::algorithms::transforms::TransformerInterface *transformer)
    {
        this->transformer = transformer;
    }
    std::array<PlateHitBinData, 10> getPeakData();

private:
    tap::algorithms::CMSISMat<8, 1> normaliseBins(tap::algorithms::CMSISMat<8, 1> mat);
    tap::algorithms::CMSISMat<8, 1> blurBins(tap::algorithms::CMSISMat<8, 1> mat);
    tap::Drivers *drivers;
    int dataTimestamp;
    float hitAngle_chassisRelative_radians;
    float hitAngle_worldRelative_radians;
    bool hitRecently;
    float lastDPS;
    int lastHitPlateID;
    tap::arch::MilliTimeout hitTimer;
    const int HIT_EXPIRE_TIME = 5;
    const uint8_t BIN_NUMBER = 8;
    tap::algorithms::CMSISMat<8, 1> bins;
    const float DECAY_FACTOR = 0.99995;
    float lastPeakAngleDegrees;
    aruwsrc::algorithms::transforms::TransformerInterface *transformer;
    const tap::algorithms::CMSISMat<8, 8> BLUR_CONVOLVE_MATRIX;
    // clang-format off
    static constexpr float BLUR_CONVOLVE_MATRIX_DATA[64] = {
        0.5 , 0.25, 0   , 0   , 0   , 0   , 0   , 0.25,
        0.25, 0.5 , 0.25, 0   , 0   , 0   , 0   , 0   ,
        0   , 0.25, 0.5 , 0.25, 0   , 0   , 0   , 0   ,
        0   , 0   , 0.25, 0.5 , 0.25, 0   , 0   , 0   ,
        0   , 0   , 0   , 0.25, 0.5 , 0.25, 0   , 0   ,
        0   , 0   , 0   , 0   , 0.25, 0.5 , 0.25, 0   ,
        0   , 0   , 0   , 0   , 0   , 0.25, 0.5 , 0.25,
        0.25, 0   , 0   , 0   , 0   , 0   , 0.25, 0.5  
    };

    // clang-format on
};

}  // namespace aruwsrc::communication::serial
#endif  // PLATE_HIT_TRACKER_HPP_