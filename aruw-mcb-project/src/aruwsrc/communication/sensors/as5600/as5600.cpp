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

AS5600::AS5600(tap::Drivers* drivers, tap::gpio::Analog::Pin pin) : drivers(drivers), pin(pin) {}

void AS5600::read()
{
	this->raw_measurement = this->drivers->analog.read(this->pin);
    this->measurement = (this->raw_measurement * 1.0f / 4096.0f) * 360.0f;
}

float AS5600::getPosition() { return measurement; }

}  // namespace aruwsrc::communication::sensors::as5600
