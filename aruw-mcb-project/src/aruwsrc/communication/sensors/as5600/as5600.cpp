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

#include "as5600.hpp"

namespace aruwsrc::communication::sensors::as5600
{

AS5600::AS5600( Config &config) : config(config)
{
}

void AS5600::update()
{
	raw_measurement = config.analog->read(config.pin);
    if(raw_measurement < config.min_millivolt){
        config.min_millivolt = raw_measurement;
    } else if (raw_measurement > config.max_millivolt){
        config.max_millivolt = raw_measurement;
    }
    measurement = raw_measurement - config.min_millivolt; // Have it read 0 if min
    measurement = measurement / (config.max_millivolt - config.min_millivolt); // Normalize
    measurement = measurement * 360; // Convert to degrees
}

float AS5600::getPosition() { return measurement; }

}  // namespace aruwsrc::communication::sensors::as5600
