/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef AS5600_HPP_
#define AS5600_HPP_

#include "tap/drivers.hpp"
#include "tap/communication/sensors/sensor_interface.hpp"

namespace aruwsrc::communication::sensors::as5600
{

class AS5600 : public tap::communication::sensors::SensorInterface
{
public:
    struct Config
    {
        tap::gpio::Analog* analog;
        tap::gpio::Analog::Pin pin;
        int min_millivolt;
        int max_millivolt;
    };

    AS5600(Config &config);

    // Reads the sensor value and updates the encoder measurement
    void update();

    // Returns the position in degrees
    float getPosition();

private:
    Config& config;

    float measurement;
    int16_t raw_measurement;
};

}  // namespace aruwsrc::communication::sensors::as5600

#endif  // AS5600_HPP_
