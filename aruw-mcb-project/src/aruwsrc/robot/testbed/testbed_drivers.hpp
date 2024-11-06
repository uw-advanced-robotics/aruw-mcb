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
#include "aruwsrc/communication/sensors/imu/ism330dlc/ism330dlc.hpp"
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "tap/board/board.hpp"

namespace aruwsrc::testbed
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : tap::Drivers(),
                ism330dlc(),
                ism330dlcTerminalSerialHandler(this, &this->ism330dlc)
    {}

public:
    aruwsrc::communication::sensors::imu::Ism330dlc<I2cMaster> ism330dlc;
    tap::communication::sensors::imu::ImuTerminalSerialHandler ism330dlcTerminalSerialHandler;


};  // class aruwsrc::TestbedDrivers
}  // namespace aruwsrc::testbed

#endif  // STANDARD_DRIVERS_HPP_
