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

#include "modm/processing/protothread/protothread.hpp"
#include "modm/processing/resumable.hpp"
#include "modm/architecture/interface/i2c_device.hpp"
#include "as5600_register.hpp"

namespace aruwsrc::communication::sensors::as5600v2 {

template<class I2cMaster>
class AS5600 : public modm::I2cDevice<I2cMaster>, modm::pt::Protothread {

public:
AS5600() : modm::I2cDevice<I2cMaster>(AS5600_ADDRESS), modm::pt::Protothread() {};


// This will setup the I2C master?? TAMU initalizes the i2c here. I think i do this in the multiplexer
// Do config here
void init();

void read();

void getPosition();

void getVelocity()


};


}  // namespace aruwsrc::communication::sensors::as5600v2

#endif  // AS5600_HPP_
