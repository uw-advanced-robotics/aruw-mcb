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
    MavlinkParserTester parser(&drivers, Uart::Uart7);

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
    uint16_t crc = crc_calculate(rawMessage + 1, 10);
    crc_accumulate(39, &crc);  // Don't forget about the magic number
    convertToLittleEndian(static_cast<uint16_t>(crc), &rawMessage[11]);

    uint16_t currByte = 0;
    ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
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

    for (int i = 0; i < 25; i++)
    {
        parser.read();
    }

    EXPECT_EQ(parser.foundHeadByte, 1);
    EXPECT_EQ(parser.CRCFailed, 0);
    EXPECT_EQ(parser.readAllOfAHeader, 1);
    EXPECT_EQ(parser.readAWholeMessage, 1);

    EXPECT_EQ(0xFD, parser.newMessage.header.frame_head_byte);
    EXPECT_EQ(1, parser.newMessage.header.payload_len);
    EXPECT_EQ(2, parser.newMessage.header.incompat_flags);
    EXPECT_EQ(3, parser.newMessage.header.compat_flags);
    EXPECT_EQ(4, parser.newMessage.header.seq);
    EXPECT_EQ(5, parser.newMessage.header.sysid);
    EXPECT_EQ(6, parser.newMessage.header.compid);
    EXPECT_EQ(30, parser.newMessage.header.msgid_value());
    EXPECT_EQ(7, parser.newMessage.payload[0]);

    EXPECT_EQ(crc, parser.newMessage.crc);
}

TEST(MavlinkParser, read_parses_message_with_bad_crc)
{
    Drivers drivers;
    MavlinkParserTester parser(&drivers, Uart::Uart7);

    uint8_t rawMessage[13];
    rawMessage[0] = 0xFD;  /// Start byte
    rawMessage[1] = 1;     /// Payload length of 1
    rawMessage[2] = 2;     /// Incompat flags
    rawMessage[3] = 3;     /// Compat flags
    rawMessage[4] = 4;     /// Sequence
    rawMessage[5] = 5;     /// System ID
    rawMessage[6] = 6;     /// Component ID
    rawMessage[7] = 30;
    rawMessage[8] = 0;
    rawMessage[9] = 0;
    rawMessage[10] = 7;  /// Payload
    // Now for CRC
    rawMessage[11] = 0x00;
    rawMessage[12] = 0x00;

    uint16_t currByte = 0;
    ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
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

    for (int i = 0; i < 25; i++)
    {
        parser.read();
    }

    EXPECT_EQ(parser.CRCFailed, 1);
}

TEST(MavlinkParser, read_parses_message_with_zero_length)
{
    Drivers drivers;
    MavlinkParserTester parser(&drivers, Uart::Uart7);

    uint8_t rawMessage[12];  // Adjusted size for 0 payload length
    rawMessage[0] = 0xFD;    /// Start byte
    rawMessage[1] = 0;       /// Payload length of 0
    rawMessage[2] = 132;     /// Incompat flags
    rawMessage[3] = 243;     /// Compat flags
    rawMessage[4] = 111;     /// Sequence
    rawMessage[5] = 222;     /// System ID
    rawMessage[6] = 123;     /// Component ID
    rawMessage[7] = 32;      /// MsgID part 1
    rawMessage[8] = 0;       /// MsgID part 2
    rawMessage[9] = 0;       /// MsgID part 3
    // Calculate CRC for header only, as payload length is 0
    uint16_t crc = crc_calculate(rawMessage + 1, 9);
    crc_accumulate(185, &crc);  // Magic number for CRC
    convertToLittleEndian(static_cast<uint16_t>(crc), &rawMessage[10]);

    uint16_t currByte = 0;
    ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t* data, std::size_t length)
            {
                std::cout << "Length : " << length << std::endl;
                std::cout << "Curr byte: " << currByte << std::endl;
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

    for (int i = 0; i < 25; i++)
    {
        parser.read();
    }

    EXPECT_EQ(parser.foundHeadByte, 1);
    EXPECT_EQ(parser.CRCFailed, 0);
    EXPECT_EQ(parser.readAllOfAHeader, 1);
    EXPECT_EQ(parser.readAWholeMessage, 1);

    EXPECT_EQ(0xFD, parser.newMessage.header.frame_head_byte);
    EXPECT_EQ(0, parser.newMessage.header.payload_len);
    EXPECT_EQ(132, parser.newMessage.header.incompat_flags);
    EXPECT_EQ(243, parser.newMessage.header.compat_flags);
    EXPECT_EQ(111, parser.newMessage.header.seq);
    EXPECT_EQ(222, parser.newMessage.header.sysid);
    EXPECT_EQ(123, parser.newMessage.header.compid);
    EXPECT_EQ(32, parser.newMessage.header.msgid_value());

    EXPECT_EQ(crc, parser.newMessage.crc);
}

// Test with big payload (100 bytes)
TEST(MavlinkParser, read_parses_message_with_big_payload)
{
    Drivers drivers;
    MavlinkParserTester parser(&drivers, Uart::Uart7);

    uint8_t rawMessage[113];
    rawMessage[0] = 0xFD;  /// Start byte
    rawMessage[1] = 100;   /// Payload length of 100
    rawMessage[2] = 2;     /// Incompat flags
    rawMessage[3] = 3;     /// Compat flags
    rawMessage[4] = 4;     /// Sequence
    rawMessage[5] = 5;     /// System ID
    rawMessage[6] = 6;     /// Component ID
    rawMessage[7] = 30;
    rawMessage[8] = 0;
    rawMessage[9] = 0;
    for (int i = 10; i < 110; i++)
    {
        rawMessage[i] = i;  /// Payload
    }
    // Now for CRC
    uint16_t crc = crc_calculate(rawMessage + 1, 110);
    crc_accumulate(39, &crc);  // Don't forget about the magic number
    convertToLittleEndian(static_cast<uint16_t>(crc), &rawMessage[110]);

    uint16_t currByte = 0;
    ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t* data, std::size_t length)
            {
                int bytesRead = 0;
                for (std::size_t i = 0; i < length; i++)
                {
                    if (currByte == sizeof(rawMessage))
                    {
                        return bytesRead;
                    }
                    else
                    {
                        data[i] = rawMessage[currByte];
                        currByte++;
                        bytesRead++;
                    }
                }
                return bytesRead;
            });

    for (int i = 0; i < 3; i++)
    {
        parser.read();
    }

    EXPECT_EQ(crc, parser.newMessage.crc);
}
