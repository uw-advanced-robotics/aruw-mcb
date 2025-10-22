/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "circle_crosshair.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
CircleCrosshair::CircleCrosshair(RefSerialTransmitter& refSerialTransmitter)
    : HudIndicator(refSerialTransmitter)
{
}

void CircleCrosshair::initialize()
{
    uint8_t crosshairName[3];

    getUnusedGraphicName(crosshairName);

    // TODO (EDU): Modify the lines below so you draw something different than a circle!!
    RefSerialTransmitter::configGraphicGenerics(
        &crosshairGraphics.graphicData,
        crosshairName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

    RefSerialTransmitter::configCircle(
        LINE_THICKNESS + 2U,  // makes circle thicker
        CRICLE_X,
        CRICLE_Y,
        CRICLE_SIZE + 1U,  // make radius larger
        &crosshairGraphics.graphicData);
}

modm::ResumableResult<void> CircleCrosshair::sendInitialGraphics()
{
    RF_BEGIN(0)

    RF_CALL(refSerialTransmitter.sendGraphic(&crosshairGraphics));

    RF_END();
}

}  // namespace aruwsrc::control::client_display
