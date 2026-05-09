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

#ifndef GAME_TIMER_HPP_
#define GAME_TIMER_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display::indicators
{
class GameTimer : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Makes a dot circle crosshair on the screen.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    GameTimer(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        tap::communication::serial::RefSerial &refSerial);

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

    void initialize() override final;

private:
    /** The X location of the center of the timer bar on the screen, in pixels. */
    static constexpr uint16_t TIMER_CENTER_X = 1870;
    /** The Y location of the center of the timer bar on the screen, in pixels. */
    static constexpr uint16_t TIMER_CENTER_Y = 630;
    /** The height of the timer bar, in pixels. */
    static constexpr uint16_t BOX_HEIGHT = 300;
    /** The width of the timer bar, in pixels. */
    static constexpr uint16_t BOX_WIDTH = 50;

    tap::communication::serial::RefSerial &refSerial;

    Tx::Graphic1Message timerBarGraphic;

    Tx::GraphicCharacterMessage timerTextGraphic;

    int timeRemainingSeconds;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // GAME_TIMER_HPP_
