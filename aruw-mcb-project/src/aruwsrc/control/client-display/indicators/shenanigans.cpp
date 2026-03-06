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

#include "shenanigans.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display::indicators
{
Shenanigans::Shenanigans(RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter)
{
}

void Shenanigans::initialize()
{
    //std::srand(std::time(0));
    //OFFSET_X = std::rand() % (SCREEN_WIDTH*7/10) - SCREEN_WIDTH/2*7/10;
    //OFFSET_Y = std::rand() % (SCREEN_HEIGHT*7/10) - SCREEN_HEIGHT/2*7/10;
    //COLOR = static_cast<Tx::GraphicColor>(std::rand() % 8);

    uint8_t shenanigansName[3];

    getUnusedGraphicName(shenanigansName);
    RefSerialTransmitter::configGraphicGenerics(
        &shenanigansGraphics.graphicData,
        shenanigansName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        COLOR);

    RefSerialTransmitter::configCharacterMsg(
        FONTSIZE,
        LINEWIDTH,
        OFFSET_X+SCREEN_WIDTH / 2,
        OFFSET_Y+SCREEN_HEIGHT / 2,
        "Shenanigans",
        &shenanigansGraphics);
}

modm::ResumableResult<void> Shenanigans::sendInitialGraphics()
{
    RF_BEGIN(0)

    RF_CALL(refSerialTransmitter.sendGraphic(&shenanigansGraphics));

    RF_END();
}

}  // namespace aruwsrc::control::client_display::indicators
