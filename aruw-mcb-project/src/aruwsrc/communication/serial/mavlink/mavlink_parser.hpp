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

#include "mavlink_messages.hpp"

using namespace tap::communication::serial;

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
public:
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
            memset(payload, 0, PAYLOAD_SIZE);
        }

        void setCRC()
        {
            crc = crc_calculate((uint8_t*)&header + 1, sizeof(header) - 1);
            crc_accumulate(payload, header.payload_len, &crc);
            uint8_t crc_extra = get_crc_extra(header.msgid);
            crc_accumulate(crc_extra, &crc);
        }

        FrameHeader header;
        uint8_t payload[PAYLOAD_SIZE];
        uint16_t crc;
    } modm_packed;

    static const uint8_t HEAD_BYTE = 0xFD;

    static constexpr uint8_t MAX_PAYLOAD_SIZE = 255;
    using ReceivedSerialMessage = MavlinkMessage<MAX_PAYLOAD_SIZE>;

    MavlinkParser(tap::Drivers* drivers, Uart::UartPort port);

    /**
     * Initializes assigned serial port to the specified baudrate
     */
    mockable void initialize();

    /**
     * Parses incoming messages. Needs to be called periodically
     */
    mockable void read();

    /**
     * Called when a complete message is received. A derived class must
     * implement this in order to handle incoming messages properly.
     *
     * @param[in] completeMessage a reference to the full message that has
     *      just been received by this class.
     */
    virtual void messageReceiveCallback(const ReceivedSerialMessage& message) = 0;

    /**
     * Sends mavlink message 511 to the drone to get messages back at a specified interval
     */
    mockable void sendIntervalCommand(uint32_t msgid, uint32_t interval)
    {
        msgid = interval;
        interval = msgid;
    };

#ifndef ENV_UNIT_TESTS
protected:
#endif

    tap::Drivers* drivers;
    Uart::UartPort port;

    enum ParsingState
    {
        HEADER_SEARCH,            /// A header byte has not yet been received.
        PROCESSING_FRAME_HEADER,  /// A header is received and the frame header is being processed.
        PROCESS_PAYLOAD,          /// The data is being processed.
        PROCESS_CRC               /// The CRC is being processed.
    };

    ParsingState state;

    /// Message in middle of being constructed.
    ReceivedSerialMessage newMessage;

    /// Most recent complete message.
    ReceivedSerialMessage mostRecentMessage;

    int currByte;

    static constexpr uint32_t BAUD_RATE = 57600;

    bool validateCRC(ReceivedSerialMessage& message);

    // DEBUG VARIABLES
    int startedParsing = 0, foundHeadByte = 0, readAllOfAHeader = 0, PayloadTooBig = 0,
        readAWholePayload = 0, readAWholeMessage = 0, CRCFailed = 0;
};
}  // namespace aruwsrc::communication::serial::mavlink

#endif  // MAVLINK_HPP_
