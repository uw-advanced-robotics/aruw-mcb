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

#include "can_serializer.hpp"

#include <array>
#include <cstdint>

#include "aruwlib/communication/can/can.hpp"

namespace aruwlib
{
namespace motorsim
{
std::array<int16_t, 4> CanSerializer::parseMessage(modm::can::Message* message)
{
    std::array<int16_t, 4> out;
    uint8_t* data = message->data;

    // Byte Smashing!
    out[0] = static_cast<int16_t>(data[0]) << 8 | static_cast<int16_t>(data[1]);
    out[1] = static_cast<int16_t>(data[2]) << 8 | static_cast<int16_t>(data[3]);
    out[2] = static_cast<int16_t>(data[4]) << 8 | static_cast<int16_t>(data[5]);
    out[3] = static_cast<int16_t>(data[6]) << 8 | static_cast<int16_t>(data[7]);
    return out;
}

modm::can::Message* CanSerializer::serializeFeedback(
    int16_t angle,
    int16_t rpm,
    int16_t current,
    uint8_t port)
{
    modm::can::Message* out =  new modm::can::Message(HEADER[port], FEEDBACK_MESSAGE_SEND_LENGTH);

    // TODO: Confirm that val & 0xFF is the correct format for lower-order byte
    out->data[0] = angle >> 8;
    out->data[1] = angle & 0xFF;
    out->data[2] = rpm >> 8;
    out->data[3] = rpm & 0xFF;
    out->data[4] = current >> 8;
    out->data[5] = current & 0xFF;
    out->data[6] = 0;  // Cannot yet simulate temperature
    out->data[7] = 0;  // Null Byte

    return out;
}

int8_t CanSerializer::idToPort(aruwlib::motor::MotorId id)
{
    for (uint32_t i = 0; i < HEADER.size(); i++)
    {
        if (id == HEADER[i])
        {
            return i;
        }
    }

    return -1;
}

}  // namespace motorsim
}  // namespace aruwlib
#endif