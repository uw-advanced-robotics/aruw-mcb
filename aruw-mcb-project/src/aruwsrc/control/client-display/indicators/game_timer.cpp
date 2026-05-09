/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "game_timer.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display::indicators
{
GameTimer::GameTimer(RefSerialTransmitter &refSerialTransmitter, RefSerial &refSerial)
    : HudIndicator(refSerialTransmitter),
      refSerial(refSerial)
{
}

modm::ResumableResult<void> GameTimer::sendInitialGraphics()
{
    RF_BEGIN(0)

    RF_CALL(refSerialTransmitter.sendGraphic(&timerBarGraphic));

    RF_CALL(refSerialTransmitter.sendGraphic(&timerTextGraphic));

    RF_END();
}

modm::ResumableResult<void> GameTimer::update()
{
    int newTimeRemainingSeconds = refSerial.getGameData().stageTimeRemaining;

    bool update = newTimeRemainingSeconds != timeRemainingSeconds;

    int seconds = 0;
    int minutes = 0;

    RF_BEGIN(1);

    if (update)
    {
        timeRemainingSeconds = newTimeRemainingSeconds;

        minutes = timeRemainingSeconds / 60;
        seconds = timeRemainingSeconds % 60;
        timerTextGraphic.msg[0] = '0' + minutes / 10;
        timerTextGraphic.msg[1] = '0' + minutes % 10;
        timerTextGraphic.msg[2] = ':';
        timerTextGraphic.msg[3] = '0' + seconds / 10;
        timerTextGraphic.msg[4] = '0' + seconds % 10;
        timerTextGraphic.msg[5] = '\0';

        timerTextGraphic.graphicData.operation = Tx::GraphicOperation::GRAPHIC_MODIFY;

        RF_CALL(refSerialTransmitter.sendGraphic(&timerTextGraphic));
    }
    RF_END();
}

void GameTimer::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configLine(
        BOX_WIDTH,
        TIMER_CENTER_X,
        TIMER_CENTER_Y + BOX_HEIGHT / 2,
        TIMER_CENTER_X,
        TIMER_CENTER_Y - BOX_HEIGHT / 2,
        &timerBarGraphic.graphicData);

    RefSerialTransmitter::configCharacterMsg(
        15,
        3,
        TIMER_CENTER_X - 15 * 2,
        TIMER_CENTER_Y + BOX_HEIGHT / 2 + 15 + 5,
        "",
        &timerTextGraphic);
}

}  // namespace aruwsrc::control::client_display::indicators
