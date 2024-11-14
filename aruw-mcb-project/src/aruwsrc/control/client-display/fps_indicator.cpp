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

#include "fps_indicator.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
FPSIndicator::FPSIndicator(RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter)
{
}

modm::ResumableResult<bool> FPSIndicator::update()
{
    uint32_t methodStartTime = tap::arch::clock::getTimeMicroseconds();
    FPSwithGraphic = 1e6 / (methodStartTime - lastTime);

    uint32_t delay = 0;

    RF_BEGIN(0);

    fpsGraphic.graphicData[0].operation = fpsGraphic.graphicData[0].operation == Tx::GRAPHIC_DELETE
                                              ? Tx::GRAPHIC_ADD
                                              : Tx::GRAPHIC_MODIFY;

    fpsGraphic.graphicData[1].operation = fpsGraphic.graphicData[1].operation == Tx::GRAPHIC_DELETE
                                              ? Tx::GRAPHIC_ADD
                                              : Tx::GRAPHIC_MODIFY;

    RefSerialTransmitter::configInteger(
        TEXT_SIZE,
        TEXT_WIDTH,
        TEXT_X,
        TEXT_Y_1,
        (int32_t)FPSwithGraphic,
        &fpsGraphic.graphicData[0]);

    RefSerialTransmitter::configInteger(
        TEXT_SIZE,
        TEXT_WIDTH,
        TEXT_X,
        TEXT_Y_2,
        (int32_t)FPSwithoutGraphic,
        &fpsGraphic.graphicData[1]);

    RF_CALL(refSerialTransmitter.sendGraphic(&fpsGraphic));

    delay = tap::arch::clock::getTimeMicroseconds() - methodStartTime;

    FPSwithoutGraphic = 1e6 / (methodStartTime - lastTime - delay);

    lastTime = methodStartTime;

    RF_END();
}

modm::ResumableResult<bool> FPSIndicator::sendInitialGraphics()
{
    RF_BEGIN(1);

    RF_CALL(refSerialTransmitter.sendGraphic(&fpsGraphic));

    RF_END();
}

void FPSIndicator::initialize()
{
    uint8_t indicatorName[3];

    for (int i = 0; i < 2; i++)
    {
        getUnusedGraphicName(indicatorName);
        RefSerialTransmitter::configGraphicGenerics(
            &fpsGraphic.graphicData[i],
            indicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            Tx::GraphicColor::ORANGE);

        RefSerialTransmitter::configInteger(
            TEXT_SIZE,
            TEXT_WIDTH,
            TEXT_X,
            i ? TEXT_Y_2 : TEXT_Y_1,
            0,
            &fpsGraphic.graphicData[i]);
    }

    lastTime = tap::arch::clock::getTimeMicroseconds();
}

}  // namespace aruwsrc::control::client_display
