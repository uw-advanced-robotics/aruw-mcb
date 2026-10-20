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

#ifndef MATCH_TIMER_INDICATOR_HPP_
#define MATCH_TIMER_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
class MatchTimerIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    explicit MatchTimerIndicator(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;
    modm::ResumableResult<void> sendInitialGraphics() override final;
    modm::ResumableResult<void> update() override final;

private:
    static constexpr uint16_t FONT_SIZE = 25;
    static constexpr uint16_t LINE_WIDTH = 2;
    static constexpr uint16_t TEXT_X = 100;
    static constexpr uint16_t TEXT_Y = 1000;

    tap::communication::serial::RefSerialData::Tx::GraphicCharacterMessage timerGraphic;
    uint32_t startTimeMs = 0;
    uint32_t elapsedSeconds = 0;
    uint32_t displayedSeconds = 0;

    void updateTimerText();
};
}  // namespace aruwsrc::control::client_display

#endif  // MATCH_TIMER_INDICATOR_HPP_
