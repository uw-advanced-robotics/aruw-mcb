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

#ifndef HOLD_FIRE_INDICATOR_HPP_
#define HOLD_FIRE_INDICATOR_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/communication/serial/sentry_response_handler.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * DRAWS AN INTEGER COUNTDOWN FOR HOLDFIRETIMER
 */
class HoldFireIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:

    /**
     * Construct a fasdifaj.
     */
    HoldFireIndicator(
        const aruwsrc::communication::serial::SentryResponseHandler& sentryResponseHandler,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    const aruwsrc::communication::serial::SentryResponseHandler& sentryResponseHandler;
    static constexpr uint16_t START_Y = 760;
    static constexpr uint16_t START_X = static_cast<int>(5 * SCREEN_WIDTH / 6);


    static constexpr Tx::GraphicColor TIMER_COLOR = Tx::GraphicColor::WHITE;
    static constexpr uint16_t TIMER_FONT_SIZE = 30;
    static constexpr uint16_t TIMER_LINE_WIDTH = 3;
    // hah
    Tx::Graphic1Message timerMessage;

    uint32_t lastVal = 0;
};
}  // namespace aruwsrc::control::client_display

#endif  //  HOLD_FIRE_INDICATOR_HPP_
