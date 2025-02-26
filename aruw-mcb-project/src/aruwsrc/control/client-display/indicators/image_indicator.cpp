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

#include "aruwsrc/control/client-display/images/marcus.hpp"

using namespace tap::communication::serial;
using namespace aruwsrc::control::client_display::images;

namespace aruwsrc::control::client_display
{
ImageIndicator::ImageIndicator(RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      image({NUM_LINES_MARCUS, MARCUS_LINES})
{
}

void ImageIndicator::initialize() { index = 0; }

modm::ResumableResult<void> ImageIndicator::update()
{
    auto currentTuple = image.lines[0];
    int startX = 0;
    int startY = 0;
    int endX = 0;
    int endY = 0;

    RF_BEGIN(1);

    if (index > image.size - 1)
    {
        RF_RETURN();
    }

    currentTuple = image.lines[index];
    startX = std::get<0>(currentTuple) + IMAGE_X_OFFSET;
    startY = std::get<1>(currentTuple) + IMAGE_Y_OFFSET;
    endX = std::get<2>(currentTuple) + IMAGE_X_OFFSET;
    endY = std::get<3>(currentTuple) + IMAGE_Y_OFFSET;

    uint8_t graphicName[3];
    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &imageGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

    RefSerialTransmitter::configLine(
        LINE_THICKNESS,
        startX,
        startY,
        endX,
        endY,
        &imageGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&imageGraphic));

    index++;
    RF_END();
}

}  // namespace aruwsrc::control::client_display
