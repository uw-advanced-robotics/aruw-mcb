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

#include "number_spam.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
NumberSpam::NumberSpam(RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter)
{
}

modm::ResumableResult<bool> NumberSpam::update()
{
    RF_BEGIN(0);

    if (!delayTimeout.execute())
    {
        RF_RETURN(false);
    }
    delayTimeout.restart(timeout);

    time = tap::arch::clock::getTimeMilliseconds();

    // Draw the number
    numberGraphic.graphicData.operation = numberGraphic.graphicData.operation == Tx::GRAPHIC_DELETE
                                              ? Tx::GRAPHIC_ADD
                                              : Tx::GRAPHIC_MODIFY;

    RefSerialTransmitter::configInteger(20, 4, TEXT_X, TEXT_Y, time, &numberGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&numberGraphic));

    RF_END();
}

modm::ResumableResult<bool> NumberSpam::sendInitialGraphics()
{
    RF_BEGIN(1);

    delayTimeout.restart(timeout);

    RF_CALL(refSerialTransmitter.sendGraphic(&numberGraphic));

    RF_END();
}

void NumberSpam::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &numberGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::ORANGE);

    RefSerialTransmitter::configInteger(20, 4, TEXT_X, TEXT_Y, 100, &numberGraphic.graphicData);
}

}  // namespace aruwsrc::control::client_display
