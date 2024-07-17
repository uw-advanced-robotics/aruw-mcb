/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MAVLINK_HPP_
#define MAVLINK_HPP_

#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::communication::serial::mavlink
{
/**
 * Generic MAVLink 2 parser that works over UART.
 *
 * Extend this class and implement messageReceiveCallback to process specific messages.
 * More info at: https://mavlink.io/en/guide/mavlink_2.html
 */
class MavlinkParser
{
    struct FrameHeader
    {
        uint8_t frame_head_byte;
        uint8_t payload_len;
        uint8_t incompat_flags;
        uint8_t compat_flags;
        uint8_t seq;
        uint8_t sysid;
        uint8_t compid;
        uint32_t msgid : 24;
    } modm_packed;

    static const uint8_t SERIAL_HEAD_BYTE = 0xFD;

    template <int PAYLOAD_SIZE>
    struct MavlinkMessage
    {
        explicit MavlinkMessage(uint8_t seq = 0, uint8_t sysid = 2, uint8_t compid = 25)
        {
            header.frame_head_byte = SERIAL_HEAD_BYTE;
            header.seq = seq;
            header.incompat_flags = 0;
            header.sysid = sysid;
            header.compid = compid;
        }

        void setCRC()
        {
            crc = 0;  // TODO
        }

        FrameHeader header;
        uint8_t payload[PAYLOAD_SIZE];
        uint16_t crc;
    } modm_packed;

    static constexpr uint8_t MAX_PAYLOAD_SIZE = 255;
    using RecievedSerialMessage = MavlinkMessage<MAX_PAYLOAD_SIZE>;
};
}  // namespace aruwsrc::communication::serial::mavlink

#endif  // MAVLINK_HPP_
