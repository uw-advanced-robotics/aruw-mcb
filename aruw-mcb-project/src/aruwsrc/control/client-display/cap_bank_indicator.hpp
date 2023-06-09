/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CAP_BANK_INDICATOR_HPP_
#define CAP_BANK_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/communication/sensors/power/external_capacitor_bank.hpp"
#include "modm/math/geometry/vector2.hpp"
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
class CapBankIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a ClientDisplayCommand.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] refSerialTransmitter Transmitter that stores ref serial transmission state for the
     * protothread that this indicator is used in.
     */
    CapBankIndicator(
        tap::Drivers &drivers,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const aruwsrc::communication::sensors::power::ExternalCapacitorBank* capBank);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** The X location of the center of the cap bank bar on the screen, in pixels. */
    static constexpr uint16_t CAP_CENTER_X = 1870;
    /** The Y location of the center of the cap bank bar on the screen, in pixels. */
    static constexpr uint16_t CAP_CENTER_Y = 630;
    /** The height of the cap bank bar, in pixels. */
    static constexpr uint16_t BOX_HEIGHT = 300;
    /** The width of the cap bank bar, in pixels. */
    static constexpr uint16_t BOX_WIDTH = 50;

    tap::Drivers &drivers;

    const aruwsrc::communication::sensors::power::ExternalCapacitorBank* capBank;

    /**
     * Two graphics that represent chassis orientation. The first graphic is the line representing
     * the turret, and the second graphic is a thick line that represents the chassis and is rotated
     * some amount to represent chassis orientation.
     */
    Tx::Graphic2Message capBankGraphics;
};
}  // namespace aruwsrc::control::client_display

#endif  //  CAP_BANK_INDICATOR_HPP_
