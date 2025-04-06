/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "virtual_can_encoder.hpp"

namespace aruwsrc::virtualMCB
{
using namespace tap::encoder;

VirtualCanEncoder::VirtualCanEncoder(
    tap::Drivers* drivers,
    CanEncoderId id,
    MCBLite* lite,
    tap::can::CanBus canBus,
    bool isInverted = false,
    float gearRatio = 1,
    uint32_t encoderHomePosition = 0)
    : CanEncoder(drivers, id, canBus, isInverted, gearRatio, encoderHomePosition)
{
    if (canBus == tap::can::CanBus::CAN_BUS1)
    {
        lite->can1Encoders[id - CanEncoderId::ID0] = this;
    }
    else
    {
        lite->can2Encoders[id - CanEncoderId::ID0] = this;
    }
}

void VirtualCanEncoder::initialize() {}

}  // namespace aruwsrc::virtualMCB
