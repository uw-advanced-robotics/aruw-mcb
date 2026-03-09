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

#ifndef VIRTUAL_ANALOG_SENSOR_HPP_
#define VIRTUAL_ANALOG_SENSOR_HPP_

#include <cstdint>

#include "tap/drivers.hpp"

#include "aruwsrc/communication/can/aruw_analog_sensor.hpp"

namespace aruwsrc::communication::mcb_lite
{
class MCBLite;

class VirtualAnalogSensor : public aruwsrc::communication::can::AruwAnalogSensor
{
    friend class MCBLite;

public:
    VirtualAnalogSensor(tap::Drivers* drivers, tap::can::CanBus canBus, uint16_t canId = 0x1D6)
        : AruwAnalogSensor(drivers, canBus, canId)
    {
    }

    void initialize() {}

    void processAnalogSensorUARTMessage(uint16_t ai0, uint16_t ai1)
    {
        modm::can::Message msg;
        msg.data[0] = ai0 & 0xFF;
        msg.data[1] = ai0 >> 8;
        msg.data[2] = ai1 & 0xFF;
        msg.data[3] = ai1 >> 8;
        processMessage(msg);
    }
};
}  // namespace aruwsrc::communication::mcb_lite

#endif  // VIRTUAL_ANALOG_SENSOR_HPP_
