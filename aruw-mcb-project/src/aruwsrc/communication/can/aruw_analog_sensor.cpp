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

#include "aruw_analog_sensor.hpp"

namespace aruwsrc::communication::can
{
AruwAnalogSensor::AruwAnalogSensor(tap::Drivers* drivers, tap::can::CanBus canBus, uint16_t canId)
    : tap::can::CanRxListener(drivers, canId, canBus),
      CAN_ID(canId)
{
}

void AruwAnalogSensor::processMessage(const modm::can::Message& message)
{
    this->heartbeat.restart(100);
    this->ai1 = message.data[1] << 8 | message.data[0];
    this->ai0 = message.data[3] << 8 | message.data[2];

    for (UpdateListener* listener : listeners)
    {
        if (listener != nullptr)
        {
            listener->onAnalogSensorUpdated();
        }
    }
}

void AruwAnalogSensor::initialize()
{
    this->attachSelfToRxHandler();
    this->heartbeat.restart(0);
}

bool AruwAnalogSensor::addUpdateListener(UpdateListener* listener)
{
    if (listener == nullptr)
    {
        return false;
    }

    for (UpdateListener* existingListener : listeners)
    {
        if (existingListener == listener)
        {
            return true;
        }
    }

    for (UpdateListener*& listenerSlot : listeners)
    {
        if (listenerSlot == nullptr)
        {
            listenerSlot = listener;
            return true;
        }
    }

    return false;
}

void AruwAnalogSensor::removeUpdateListener(const UpdateListener* listener)
{
    for (UpdateListener*& listenerSlot : listeners)
    {
        if (listenerSlot == listener)
        {
            listenerSlot = nullptr;
        }
    }
}
}  // namespace aruwsrc::communication::can
