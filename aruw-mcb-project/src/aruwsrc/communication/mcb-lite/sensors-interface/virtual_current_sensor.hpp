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
#ifndef VIRTUAL_CURRENT_SENSOR_HPP_
#define VIRTUAL_CURRENT_SENSOR_HPP_

#include "tap/communication/sensors/current/current_sensor_interface.hpp"

namespace aruwsrc::virtualMCB
{

class VirtualCurrentSensor : public tap::communication::sensors::current::CurrentSensorInterface
{
    friend class SerialMCBLite;

public:
    VirtualCurrentSensor() {}

    // Please don't call this
    void update() override {}

    float getCurrentMa() const override { return current; }

private:
    float current;
};

}  // namespace aruwsrc::virtualMCB

#endif
