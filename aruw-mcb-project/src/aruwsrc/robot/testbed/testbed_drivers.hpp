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

#ifndef TESTBED_DRIVERS_HPP_
#define TESTBED_DRIVERS_HPP_

#include "tap/drivers.hpp"

#include "aruwsrc/communication/sensors/as5600/as5600.hpp"

namespace aruwsrc::testbed
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : tap::Drivers(), as5600(config) {}

public:
    aruwsrc::communication::sensors::as5600::AS5600::Config config = {
        .analog = &(this->analog),
        .pin = tap::gpio::Analog::Pin::U};

    aruwsrc::communication::sensors::as5600::AS5600 as5600;

};  // class aruwsrc::StandardDrivers
}  // namespace aruwsrc::testbed

#endif  // STANDARD_DRIVERS_HPP_
