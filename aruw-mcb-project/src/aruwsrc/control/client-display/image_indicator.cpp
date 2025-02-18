/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "image_indicator.hpp"

#include "images/marcus.hpp"

using namespace tap::communication::serial;
using namespace aruwsrc::control::client_display::images;

namespace aruwsrc::control::client_display
{

ImageIndicator::ImageIndicator(RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter)
{
}

void ImageIndicator::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &imageGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

    index = 0;
}

modm::ResumableResult<bool> ImageIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    RF_END_RETURN(true);
}

modm::ResumableResult<bool> ImageIndicator::update()
{
    RF_BEGIN(0)

    auto currentTuple = MARCUS_LINES[index];
    int startX = std::get<0>(currentTuple);
    int startY = std::get<1>(currentTuple);
    int endX = std::get<2>(currentTuple);
    int endY = std::get<3>(currentTuple);

    RefSerialTransmitter::configLine(
        LINE_THICKNESS,
        startX + IMAGE_X_OFFSET,
        startY + IMAGE_Y_OFFSET,
        endX + IMAGE_X_OFFSET,
        endY + IMAGE_Y_OFFSET,
        &imageGraphic.graphicData);
    
    RF_CALL(refSerialTransmitter.sendGraphic(&imageGraphic));

    index = (index + 1) % NUM_LINES_MARCUS;

    RF_END_RETURN(true);
}




}  // namespace aruwsrc::control::client_display
