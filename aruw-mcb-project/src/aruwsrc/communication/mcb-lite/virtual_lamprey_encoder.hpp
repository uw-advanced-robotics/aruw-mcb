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

#ifndef VIRTUAL_LAMPREY_ENCODER_HPP_
#define VIRTUAL_LAMPREY_ENCODER_HPP_

#include "aruwsrc/communication/sensors/encoder/lamprey_encoder.hpp"

#include "mcb_lite.hpp"
#include "virtual_can_encoder.hpp"

namespace aruwsrc::communication::mcb_lite
{

class VirtualLampreyEncoder : public VirtualCanEncoder, public sensors::encoder::LampreyEncoder
{
public:
    VirtualLampreyEncoder(
        tap::Drivers* drivers,
        tap::encoder::CanEncoderId id,
        MCBLite* lite,
        tap::can::CanBus canBus,
        const modm::Pair<float, float> (&emptyLut)[0],
        bool isInverted = false,
        float gearRatio = 1,
        uint32_t encoderHomePosition = 0)
        : VirtualCanEncoder(drivers, id, lite, canBus, isInverted, gearRatio, encoderHomePosition),
          LampreyEncoder(drivers, id, canBus, emptyLut, isInverted)
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

    template <std::size_t LUT_SIZE>
    VirtualLampreyEncoder(
        tap::Drivers* drivers,
        tap::encoder::CanEncoderId id,
        MCBLite* lite,
        tap::can::CanBus canBus,
        const modm::Pair<float, float> (&lookupTableConfig)[LUT_SIZE],
        bool isInverted = false,
        float gearRatio = 1,
        uint32_t encoderHomePosition = 0)
        : VirtualCanEncoder(drivers, id, lite, canBus, isInverted, gearRatio, encoderHomePosition),
          LampreyEncoder(drivers, id, canBus, lookupTableConfig, isInverted)
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

    void initialize() override {}

    DISALLOW_COPY_AND_ASSIGN(VirtualLampreyEncoder)
};

}  // namespace aruwsrc::communication::mcb_lite

#endif  // VIRTUAL_LAMPREY_ENCODER_HPP_
