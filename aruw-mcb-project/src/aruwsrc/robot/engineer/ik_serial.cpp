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

#include "ik_serial.hpp"

#include <cassert>

namespace aruwsrc::engineer
{
void IKSerial::messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
{
    if (completeMessage.messageType == IK_SOLUTION_MESSAGE_ID)
    {
        memcpy(&ikSolution, completeMessage.data, sizeof(IKSolution));
    }
    else if (completeMessage.messageType == SLOT_DELTA_MESSAGE_ID)
    {
        memcpy(&slotDelta, completeMessage.data, sizeof(SlotDelta));
    }
}

void IKSerial::sendJointPositions()
{
    if (superstructure == nullptr)
    {
        return;
    }

    aruwsrc::engineer::arm::Position position = superstructure->getPosition();
    memcpy(positionMessage.data, &position, sizeof(position));
    positionMessage.setCRC16();
    drivers->uart.write(
        port,
        reinterpret_cast<uint8_t *>(&positionMessage),
        sizeof(positionMessage));
}
}  // namespace aruwsrc::engineer
