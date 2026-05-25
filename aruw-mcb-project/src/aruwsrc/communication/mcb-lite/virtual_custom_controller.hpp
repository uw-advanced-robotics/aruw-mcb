/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VIRTUAL_CUSTOM_CONTROLLER_HPP_
#define VIRTUAL_CUSTOM_CONTROLLER_HPP_

#include "tap/communication/serial/dji_serial.hpp"

#include "message_types.hpp"

namespace aruwsrc::communication::mcb_lite
{
class VirtualCustomController
{
    friend class MCBLite;

public:
    const uint8_t* getData() const { return data; }

private:
    void processCustomControllerMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        memcpy(data, completeMessage.data, sizeof(CustomControllerMessage));
    }

    uint8_t data[30] = {};
};
}  // namespace aruwsrc::communication::mcb_lite
#endif