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

#include "hud_indicator.hpp"

#include <cstdint>

namespace aruwsrc::control::client_display
{
uint32_t HudIndicator::currGraphicName = 0;

HudIndicator::HudIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : refSerialTransmitter(refSerialTransmitter)
{
}

void HudIndicator::resetGraphicNameGenerator() { currGraphicName = 0; }

std::optional<std::array<uint8_t, 3>> HudIndicator::getUnusedGraphicName()
{
    if (currGraphicName > MAX_GRAPHIC_NAME)
    {
        return std::nullopt;
    }
    else
    {
        std::optional<std::array<uint8_t, 3>> graphicName;
        (*graphicName)[0] = static_cast<uint8_t>((currGraphicName >> 16) & 0xff);
        (*graphicName)[1] = static_cast<uint8_t>((currGraphicName >> 8) & 0xff);
        (*graphicName)[2] = static_cast<uint8_t>(currGraphicName & 0xff);
        currGraphicName++;
        return graphicName;
    }
}

}  // namespace aruwsrc::control::client_display
