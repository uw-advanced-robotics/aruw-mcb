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

#ifndef NUMBER_SPAM_HPP_
#define NUMBER_SPAM_HPP_

#include "tap/architecture/periodic_timer.hpp"

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
/**
 * Spams numbers on the screen.
 */
class NumberSpam : public HudIndicator, protected modm::Resumable<3>
{
public:
    /**
     * Construct a NumberSpam object.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    NumberSpam(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

private:
    // X position of the text
    static constexpr uint16_t TEXT_X = SCREEN_WIDTH / 2;
    // Y position of the text
    static constexpr uint16_t TEXT_Y = 500;

    Tx::Graphic1Message numberGraphic;

    uint32_t time;

    tap::arch::PeriodicMilliTimer delayTimeout;
    uint32_t timeout = 500;

};

}  // namespace aruwsrc::control::client_display

#endif  // NUMBER_SPAM_HPP_
