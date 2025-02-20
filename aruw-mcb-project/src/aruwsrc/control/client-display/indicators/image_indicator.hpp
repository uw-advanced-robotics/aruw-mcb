/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef IMAGE_INDICATOR_HPP_
#define IMAGE_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "images/image.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{

class ImageIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    ImageIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<bool> update() override final;

private:
    int index = 0;

    static constexpr uint16_t IMAGE_X_OFFSET = 600;
    static constexpr int16_t IMAGE_Y_OFFSET = -400;

    static constexpr uint16_t LINE_THICKNESS = 1;

    Tx::Graphic1Message imageGraphic;

    images::Image image;
};

}  // namespace aruwsrc::control::client_display

#endif
