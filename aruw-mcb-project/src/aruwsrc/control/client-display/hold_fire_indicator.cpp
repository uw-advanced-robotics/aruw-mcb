/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "hold_fire_indicator.hpp"

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
HoldFireIndicator::HoldFireIndicator(
    const aruwsrc::communication::serial::SentryResponseHandler& sentryResponseHandler,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      sentryResponseHandler(sentryResponseHandler)
{
}

// void RefSerialTransmitter::configInteger(

modm::ResumableResult<bool> HoldFireIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);
    RF_CALL(refSerialTransmitter.sendGraphic(&timerMessage));
    RF_END();
}

modm::ResumableResult<bool> HoldFireIndicator::update()
{
    RF_BEGIN(1);
    timerMessage.graphicData.value = sentryResponseHandler.getHoldFireTimeRemainingSec();
    RF_CALL(refSerialTransmitter.sendGraphic(&timerMessage));
    RF_END();
}


void HoldFireIndicator::initialize()
{
    uint8_t name[3];
    getUnusedGraphicName(name);

    RefSerialTransmitter::configGraphicGenerics(
        &timerMessage.graphicData,
        name,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        TIMER_COLOR);


    // center of the reticle, in pixels
    // paste it in the center of the screen :) enjoy!
    uint16_t msgX = static_cast<int>(SCREEN_WIDTH / 2);
    uint16_t msgY = static_cast<int>(SCREEN_HEIGHT / 2);

    // y coordinate of the horizontal reticle line

    RefSerialTransmitter::configInteger(
        TIMER_FONT_SIZE,
        10, // @todo IDK what width means
        msgX,
        msgY,
        sentryResponseHandler.getHoldFireTimeRemainingSec(),
        &timerMessage.graphicData);
    }

}  // namespace aruwsrc::control::client_display