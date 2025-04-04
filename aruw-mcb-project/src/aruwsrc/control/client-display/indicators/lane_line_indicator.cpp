/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "lane_line_indicator.hpp"

namespace aruwsrc::control::client_display
{
LaneLineIndicator::LaneLineIndicator(
    RefSerialTransmitter &refSerialTransmitter,
    const Transform &worldToCameraTransform)
    : HudIndicator(refSerialTransmitter),
      worldToCameraTransform(worldToCameraTransform)
{
}

void LaneLineIndicator::initialize()
{
    uint8_t indicatorName[3];

    for (int i = 0; i < 2; i++)
    {
        getUnusedGraphicName(indicatorName);
        RefSerialTransmitter::configGraphicGenerics(
            &laneLineGraphic.graphicData[i],
            indicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            Tx::GraphicColor::GREEN);
    }
}



}  // namespace aruwsrc::control::client_display
