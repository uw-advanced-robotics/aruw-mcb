/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SH1106_HPP
#error "Don't include this file directly, use 'sh1106.hpp' instead!"
#endif

#include <iostream>

template <unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<Width, Height, Flipped>::update()
{
    // no-op
    for (uint8_t y = 0; y < Height; ++y)
    {
        // command mode

        // switch to data mode
        for (uint8_t x = 0; x < Width; ++x)
        {
            uint8_t col = y / 8;
            uint8_t remainder = 1u << (y % 8);

            bool dark = (this->display_buffer[x][col] & remainder) == remainder;
            std::cout << (dark ? "\u25A0": " ");
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

template <unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<Width, Height, Flipped>::setInvert(bool invert)
{
    // no-op
}

// ----------------------------------------------------------------------------
template <unsigned int Width, unsigned int Height, bool Flipped>
void aruwlib::display::Sh1106<Width, Height, Flipped>::initializeBlocking()
{
    this->clear();
    this->update();
}
