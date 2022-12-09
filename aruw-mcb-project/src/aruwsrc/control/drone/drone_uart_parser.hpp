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
#ifndef DRONE_UART_PARSER_HPP_
#define DRONE_UART_PARSER_HPP_

#define MSG_START 0xFE
#define MSG_ID 32
#define MSG_PAYLOAD_LENGTH 28
#define MSG_CRC 185

#include <stdint.h>

#include "tap/communication/serial/uart.hpp"
#include "taproot/modm/src/modm/io/iodevice.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc
{
namespace drone
{

struct Telemetry
{
    uint32_t timestamp; /*< [ms] Timestamp (time since system boot).*/
    float x;            /*< [m] X Position*/
    float y;            /*< [m] Y Position*/
    float z;            /*< [m] Z Position*/
    float vx;           /*< [m/s] X Speed*/
    float vy;           /*< [m/s] Y Speed*/
    float vz;           /*< [m/s] Z Speed*/
};

enum MessageStatus
{
    OK,
    INCORRECT_START,
    INCORRECT_PAYLOAD_LENGTH,
    INCORRECT_MESSAGE_ID,
    INCORRECT_CRC
};

class DroneUartParser
{
public:
    DroneUartParser(Drivers *drivers);

    MessageStatus update();

    Telemetry telemetryData;

private:
    // TODO: this is what is used in terminal device, need to figure out what this should be
    static constexpr uint32_t UART_BAUD_RATE = 115200;

    Drivers *drivers;

    // TODO: This needs to be defined
    static constexpr Uart::UartPort DRONE_PORT = Uart::Uart1;

    void initialize();

    bool read(char &c);

    bool read(auto &c, int length);
};
}  // namespace drone
}  // namespace aruwsrc
#endif
