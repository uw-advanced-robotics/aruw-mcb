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

#include "hud_indicator.hpp"

#include <cstdint>

namespace aruwsrc::control::client_display
{
uint32_t HudIndicator::currListName = 0;

void HudIndicator::resetListNameGenerator() { currListName = 0; }

void HudIndicator::getUnusedListName(uint8_t listName[3])
{
    if (currListName > 0xffffff)
    {
        return;
    }
    else
    {
        listName[0] = static_cast<uint8_t>((currListName >> 16) & 0xff);
        listName[1] = static_cast<uint8_t>((currListName >> 8) & 0xff);
        listName[2] = static_cast<uint8_t>(currListName & 0xff);
        currListName++;
    }
}

}  // namespace aruwsrc::control::client_display
