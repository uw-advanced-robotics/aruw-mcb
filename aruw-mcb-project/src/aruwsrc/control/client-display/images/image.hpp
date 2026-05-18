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

#ifndef IMAGE_HPP_
#define IMAGE_HPP_

#include <cstdint>
#include <tuple>

namespace aruwsrc::control::client_display::images
{
using LineTuple = std::tuple<int16_t, int16_t, int16_t, int16_t>;

struct Image
{
    int size;
    const LineTuple *lines;
    const uint16_t IMAGE_X_OFFSET;
    const int16_t IMAGE_Y_OFFSET;
    const float IMAGE_SCALE;
};

};  // namespace aruwsrc::control::client_display::images

#endif  // IMAGE_HPP_
