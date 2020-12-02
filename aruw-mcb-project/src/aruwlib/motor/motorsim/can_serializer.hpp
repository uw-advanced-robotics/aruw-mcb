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

#ifdef PLATFORM_HOSTED
#ifndef can_serializer_hpp_

#define can_serializer_hpp_

#include <cstdint>
#include <array>
#include "modm/architecture/interface/can_message.hpp"
#include "aruwlib/communication/can/can.hpp"

namespace aruwlib
{
namespace motorsim
{
class CanSerializer
{
public:
    static std::array<int16_t, 4> parseMessage(modm::can::Message* message);

    static modm::can::Message* serializeFeedback(int16_t angle, int16_t rpm, int16_t current);
};
}
}
#endif
#endif