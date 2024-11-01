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

#ifndef CIRCLE_CROSSHAIR_HPP_
#define CIRCLE_CROSSHAIR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
class CicleCrosshair : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a CicleCrosshair object.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    CicleCrosshair(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

private:
    // X position of the circle
    static constexpr uint16_t CRICLE_X = SCREEN_WIDTH / 2;
    // Y position of the circle
    static constexpr uint16_t CRICLE_Y = SCREEN_HEIGHT / 2;
    // SIZE of the circle
    static constexpr uint16_t CRICLE_SIZE = 4;
    // Thickness of the line
    static constexpr uint16_t LINE_THICKNESS = 10;

    Tx::Graphic1Message crosshairGraphics;
};

}  // namespace aruwsrc::control::client_display

#endif  // CIRCLE_CROSSHAIR_HPP_
