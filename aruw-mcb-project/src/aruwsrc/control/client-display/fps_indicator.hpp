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

#ifndef FPS_INDICATOR_HPP_
#define FPS_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
class FPSIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Shows FPS on screen
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    FPSIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

private:
    static constexpr uint16_t TEXT_X = 5;
    static constexpr uint16_t TEXT_Y_1 = 1000;
    static constexpr uint16_t TEXT_Y_2 = TEXT_Y_1 - 50;
    static constexpr uint16_t TEXT_WIDTH = 2;
    static constexpr uint16_t TEXT_SIZE = 15;

    Tx::Graphic2Message fpsGraphic;

    float FPSwithGraphic = 0;
    float FPSwithoutGraphic = 0;
    uint32_t lastTime = 0;
};

}  // namespace aruwsrc::control::client_display

#endif  // FPS_INDICATOR_HPP_
