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
#include "aruwsrc/control/client-display/images/nathaniel_sussy.hpp"

using namespace tap::communication::serial;
using namespace aruwsrc::control::client_display::images;

namespace aruwsrc::control::client_display::indicators
{
ImageIndicator::ImageIndicator(RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      images({nathaniel_sussy, marcus})
{
}

void ImageIndicator::initialize()
{
    image_index = 0;
    line_index = 0;
}

modm::ResumableResult<void> ImageIndicator::update()
{
    auto currentImage = images[image_index];
    auto currentTuple = currentImage.lines[0];  // 0 here as placeholder until we access later
    int startX, startY, endX, endY;

    RF_BEGIN(1);

    if (image_index > images.size() - 1)
    {
        RF_RETURN();
    }

    if (line_index > currentImage.size - 1)
    {
        image_index++;
        line_index = 0;
        RF_RETURN();
    }

    currentTuple = currentImage.lines[line_index];
    startX = std::get<0>(currentTuple) * currentImage.IMAGE_SCALE + currentImage.IMAGE_X_OFFSET;
    startY = std::get<1>(currentTuple) * currentImage.IMAGE_SCALE + currentImage.IMAGE_Y_OFFSET;
    endX = std::get<2>(currentTuple) * currentImage.IMAGE_SCALE + currentImage.IMAGE_X_OFFSET;
    endY = std::get<3>(currentTuple) * currentImage.IMAGE_SCALE + currentImage.IMAGE_Y_OFFSET;

    uint8_t graphicName[3];
    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &imageGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        image_index,
        Tx::GraphicColor::GREEN);

    RefSerialTransmitter::configLine(
        LINE_THICKNESS,
        startX,
        startY,
        endX,
        endY,
        &imageGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&imageGraphic));

    line_index++;
    RF_END();
}

}  // namespace aruwsrc::control::client_display::indicators
