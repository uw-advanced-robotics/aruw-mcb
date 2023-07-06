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
    if (sentryResponseHandler.getHoldFireTimeRemainingSec() != lastVal) {
        timerMessage.graphicData.value = sentryResponseHandler.getHoldFireTimeRemainingSec();
        timerMessage.graphicData.operation = Tx::GRAPHIC_MODIFY;
        lastVal = sentryResponseHandler.getHoldFireTimeRemainingSec();
        RF_CALL(refSerialTransmitter.sendGraphic(&timerMessage));
    }
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

    RefSerialTransmitter::configInteger(
        TIMER_FONT_SIZE,
        TIMER_LINE_WIDTH, // @todo IDK what width means
        START_X,
        START_Y,
        timerMessage.graphicData.value = sentryResponseHandler.getHoldFireTimeRemainingSec(),
        &timerMessage.graphicData);
    }

}  // namespace aruwsrc::control::client_display