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

#include <gtest/gtest.h>

#include "tap/architecture/endianness_wrappers.hpp"

#include "aruwsrc/communication/serial/mavlink/mavlink_telemetry.hpp"

using namespace aruwsrc::communication::serial::mavlink;
using namespace tap::communication::serial;
using namespace tap::arch;
using namespace tap::algorithms;
using namespace testing;
using namespace tap;

class MavlinkParserTester : public MavlinkParser
{
public:
    MavlinkParserTester(tap::Drivers* drivers, Uart::UartPort port) : MavlinkParser(drivers, port)
    {
    }

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override
    {
        lastMsg = completeMessage;
    }

    ReceivedSerialMessage lastMsg;
};

TEST(MavlinkParser, read_parses_message)
{
    Drivers drivers;
    MavlinkParserTester parser(&drivers, Uart::UartPort::Uart1);

    uint8_t rawMessage[13];
    rawMessage[0] = 0xFD;  /// Start byte
    rawMessage[1] = 1;     /// Payload length of 1
    rawMessage[2] = 2;     /// Incompat flags
    rawMessage[3] = 3;     /// Compat flags
    rawMessage[4] = 4;     /// Sequence
    rawMessage[5] = 5;     /// System ID
    rawMessage[6] = 6;     /// Component ID
    // Ok this part is gonna kill me, it's a 24 bit number, so it's 3 bytes
    // Need to show 30, which is only 1 byte
    rawMessage[7] = 30;
    rawMessage[8] = 0;
    rawMessage[9] = 0;
    rawMessage[10] = 7;  /// Payload
    // Now for CRC
    uint16_t crc = crc_calculate(&rawMessage[1], 10);
    convertToLittleEndian(static_cast<uint16_t>(crc), &rawMessage[11]);

    uint16_t currByte = 0;

    ON_CALL(drivers.uart, read(Uart::Uart1, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t* data, std::size_t length)
            {
                if (length == 0)
                {
                    return 0;
                }

                if (currByte >= sizeof(rawMessage))
                {
                    return 0;
                }
                *data = rawMessage[currByte];
                currByte++;
                return 1;
            });

    for (int i = 0; i < 100; i++)
    {
        parser.read();
    }

    EXPECT_EQ(parser.foundHeadByte, 1);
    EXPECT_EQ(parser.PayloadTooBig, 0);
    EXPECT_EQ(parser.CRCFailed, 0);

    EXPECT_EQ(0xFD, parser.lastMsg.header.frame_head_byte);
    EXPECT_EQ(1, parser.lastMsg.header.payload_len);
    EXPECT_EQ(2, parser.lastMsg.header.incompat_flags);
    EXPECT_EQ(3, parser.lastMsg.header.compid);
    EXPECT_EQ(4, parser.lastMsg.header.seq);
    EXPECT_EQ(5, parser.lastMsg.header.sysid);
    EXPECT_EQ(6, parser.lastMsg.header.compid);
    EXPECT_EQ(30, parser.lastMsg.header.msgid);
    EXPECT_EQ(7, parser.lastMsg.payload[0]);

    EXPECT_EQ(crc, parser.lastMsg.crc);
}
