/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "match_timer_indicator.hpp"

#include <cstdio>
#include <cstring>

#include "tap/architecture/clock.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
MatchTimerIndicator::MatchTimerIndicator(RefSerialTransmitter& refSerialTransmitter)
    : HudIndicator(refSerialTransmitter)
{
}

void MatchTimerIndicator::initialize()
{
    uint8_t graphicName[3];
    getUnusedGraphicName(graphicName);

    RefSerialTransmitter::configGraphicGenerics(
        &timerGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

    startTimeMs = tap::arch::clock::getTimeMilliseconds();
    displayedSeconds = 0;
    updateTimerText();

    RefSerialTransmitter::configCharacterMsg(
        FONT_SIZE,
        LINE_WIDTH,
        TEXT_X,
        TEXT_Y,
        timerGraphic.msg,
        &timerGraphic);
}

modm::ResumableResult<void> MatchTimerIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&timerGraphic));

    RF_END();
}

modm::ResumableResult<void> MatchTimerIndicator::update()
{
    RF_BEGIN(1);

    elapsedSeconds = (tap::arch::clock::getTimeMilliseconds() - startTimeMs) / 1000;

    if (elapsedSeconds != displayedSeconds)
    {
        displayedSeconds = elapsedSeconds;
        timerGraphic.graphicData.operation = Tx::GRAPHIC_MODIFY;
        updateTimerText();
        RF_CALL(refSerialTransmitter.sendGraphic(&timerGraphic));
    }

    RF_END();
}

void MatchTimerIndicator::updateTimerText()
{
    snprintf(
        timerGraphic.msg,
        sizeof(timerGraphic.msg),
        "TIME: %u",
        static_cast<unsigned int>(displayedSeconds));
    timerGraphic.graphicData.endAngle = strlen(timerGraphic.msg);
}
}  // namespace aruwsrc::control::client_display
