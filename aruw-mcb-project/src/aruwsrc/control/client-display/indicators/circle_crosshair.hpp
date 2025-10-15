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
class CircleCrosshair : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Makes a dot circle crosshair on the screen.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    CircleCrosshair(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

private:
    // Offset from the center of the screen for the crosshair
#ifdef TARGET_STANDARD_NULL
    static constexpr int16_t OFFSET_X = 25;
    static constexpr int16_t OFFSET_Y = -75;
#elif defined(TARGET_STANDARD_VOID)
    static constexpr int16_t OFFSET_X = -5;
    static constexpr int16_t OFFSET_Y = -50;
#elif defined(TARGET_HERO_ZERO)
    static constexpr int16_t OFFSET_X = 2;
    static constexpr int16_t OFFSET_Y = -42;
#else
    static constexpr int16_t OFFSET_X = 0;
    static constexpr int16_t OFFSET_Y = 0;
#endif

    // TODO (EDU): Modify these constants or add new ones to mess with your new graphic!
    // X position of the circle
    static constexpr uint16_t CRICLE_X = SCREEN_WIDTH / 2 + OFFSET_X;
    // Y position of the circle
    static constexpr uint16_t CRICLE_Y = SCREEN_HEIGHT / 2 + OFFSET_Y;
    // SIZE of the circle
    static constexpr uint16_t CRICLE_SIZE = 2;
// Thickness of the line
#if defined(TARGET_HERO_ZERO)
    static constexpr uint16_t LINE_THICKNESS = 10;
#else
    static constexpr uint16_t LINE_THICKNESS = 5;
#endif

    Tx::Graphic1Message crosshairGraphics;
};

}  // namespace aruwsrc::control::client_display

#endif  // CIRCLE_CROSSHAIR_HPP_
