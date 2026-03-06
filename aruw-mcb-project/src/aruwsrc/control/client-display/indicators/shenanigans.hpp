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

#ifndef SHENANIGANS_HPP_
#define SHENANIGANS_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

#include <cstdint>
#include <ctime>

namespace aruwsrc::control::client_display::indicators
{
class Shenanigans : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Makes a shenanigans on the screen.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    Shenanigans(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

private:
    static constexpr int SCREEN_WIDTH = 1920;
    static constexpr int SCREEN_HEIGHT = 1080;

    static constexpr int16_t OFFSET_X = 0;
    static constexpr int16_t OFFSET_Y = 0;
    static constexpr Tx::GraphicColor COLOR = Tx::GraphicColor::YELLOW;

    static constexpr uint16_t FONTSIZE = 50;
    static constexpr uint16_t LINEWIDTH = 5;

    float value = 0;

    Tx::GraphicCharacterMessage shenanigansGraphics;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // SHENANIGANS_HPP_
