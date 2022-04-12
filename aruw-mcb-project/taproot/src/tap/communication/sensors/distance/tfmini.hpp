/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_TFMINI_HPP_
#define TAPROOT_TFMINI_HPP_

#include "tap/communication/serial/uart.hpp"

namespace tap::communication::sensors::distance
{
/**
 * For communicating with TFMini LiDar sensor. Although apparently it's
 * not actually lidar :)
 */
template <serial::Uart::UartPort uartPort>
class TFMini
{
public:
    TFMini()
    {
        serial::Uart::init<uartPort, 115200>();
    }

    void processNextByte(uint8_t byte)
    {
        uint8_t data;
        serial::Uart::read(uartPort, &data);
        if (sequenceNum < 2) {
            // we wait for two 0x59's in a row as that indicates
            // start of message
            if (data == 0x59) {
                buffer[sequenceNum] = data;
                sequenceNum++;
            }
        } else {
            buffer[sequenceNum] = data;
            sequenceNum++;
        }

        if (sequenceNum == 9) {
            parseBuffer();
            sequenceNum = 0;
        }
    }

    void parseBuffer()
    {
        dist = buffer[2] | buffer[3] << 8;
        strength = buffer[4] | buffer[5] << 8;
        integrationTime = buffer[6];

        // TODO: checksum logic
    }

private:
    bool awaitingHeader = true;
    bool headerCount = 0;
    int sequenceNum = 0;

    uint8_t buffer[9];

    uint16_t dist = 0;
    uint16_t strength = 0;
    uint8_t integrationTime = 0;
};

} // namespace tap::communication::sensors::distance

#endif // TAPROOT_TFMINI_HPP_
