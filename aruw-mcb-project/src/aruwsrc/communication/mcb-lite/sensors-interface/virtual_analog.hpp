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

#ifndef VIRTUAL_ANALOG_HPP_
#define VIRTUAL_ANALOG_HPP_

#include "tap/communication/gpio/analog.hpp"

namespace aruwsrc::virtualMCB
{

class VirtualAnalog : public tap::gpio::Analog
{
    friend class SerialMCBLite;

public:
    VirtualAnalog() : Analog(){};

    // Please don't call this, shouldn't be used
    void init(){};

    // Returns 0 if pin is not initialized, or data has not been received
    uint16_t read(Analog::Pin pin) const;

#ifndef ENV_UNIT_TESTS
private:
#endif
    uint16_t SPinValue, TPinValue, UPinValue, VPinValue, OLEDPinValue;
};

}  // namespace aruwsrc::virtualMCB

#endif
