/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_uart_parser.hpp"

namespace aruwsrc
{
namespace drone
{
DroneUartParser::DroneUartParser(Drivers *drivers) : drivers(drivers) {}

void DroneUartParser::initialize() {
	drivers -> uart.init<DRONE_PORT, UART_BAUDE_RATE>();
}

bool DroneUartParser::read(char &c) {
    return drivers->uart.read(DRONE_PORT, &reinterpret_cast<uint8_t &>(c));
}

void DroneUartParser::write(char c) { drivers->uart.write(DRONE_PORT, c); }

void DroneUartParser::flush() { drivers->uart.flushWriteBuffer(DRONE_PORT);}

}  // namespace drone
}  // namespace aruwsrc
