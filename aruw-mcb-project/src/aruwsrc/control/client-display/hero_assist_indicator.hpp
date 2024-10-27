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

#ifndef HERO_ASSIST_INDICATOR_HPP_
#define HERO_ASSIST_INDICATOR_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

#include "tap/communication/serial/ref_serial.hpp"

namespace aruwsrc::control::client_display
{
/**
 * Adds text to show in bright yellow the number of bullets currently the robot has
 */
class HeroAssistIndicator : public HudIndicator, protected modm::Resumable<3>
{
public:
    /**
     * Construct a HeroAssistIndicator object.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    HeroAssistIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
                        const tap::communication::serial::RefSerial &refSerial);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    // X position of the text
    static constexpr uint16_t TEXT_X = 1300;
    // Y position of the text
    static constexpr uint16_t TEXT_Y = 950;
    // WIDTH of the text
    static constexpr uint16_t TEXT_WIDTH = 15;

    Tx::GraphicCharacterMessage bulletsRemainingGraphics;

    int lastBullets = -1;

    const tap::communication::serial::RefSerial &refSerial;

};

}  // namespace aruwsrc::control::client_display

#endif  // HERO_ASSIST_INDICATOR_HPP_
