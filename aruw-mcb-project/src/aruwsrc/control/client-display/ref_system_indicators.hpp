/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef REF_SYSTEM_INDICATORS_HPP_
#define REF_SYSTEM_INDICATORS_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * Draws a bar on the HUD that represents the current charge of the cap bank.
 */
class RefSystemIndicators : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct RefSystemIndicators.
     *
     * @param[in] refSerialTransmitter Transmitter that stores ref serial transmission state for the
     * protothread that this indicator is used in.
     * @param[in] refSerial A reference to the ref serial object to get ref data.
     */
    RefSystemIndicators(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const tap::communication::serial::RefSerial &refSerial);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** The X location of the hero ammo count on the screen, in pixels. */
    static constexpr uint16_t AMMO_COUNT_CENTER_X = 1475;
    /** The Y location of the hero ammo count on the screen, in pixels. */
    static constexpr uint16_t AMMO_COUNT_CENTER_Y = 600;

    static constexpr uint16_t AMMO_COUNT_CHAR_SIZE = 70;

    const tap::communication::serial::RefSerial &refSerial;

    /**
     * A graphic that shows the amount of hero shots remaining.
     */
    Tx::GraphicCharacterMessage heroAmmoCountTextGraphic;
};
}  // namespace aruwsrc::control::client_display

#endif  //  CAP_BANK_INDICATOR_HPP_