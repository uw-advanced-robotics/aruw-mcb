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

#include <gtest/gtest.h>

#include "tap/algorithms/crc.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/mavlink_receiver.hpp"

using namespace tap::communication::serial;
using namespace tap::arch;
using namespace testing;
using namespace tap;
using namespace tap::algorithms;
using namespace aruwsrc::communication::serial;

class MavlinkReceiverTester : public MavlinkReceiver
{
public:
    MavlinkReceiverTester(Drivers *drivers, Uart::UartPort port) : MavlinkReceiver(drivers, port) {}

    void messageReceiveCallback(const ReceivedMavlinkMessage &completeMessage) override
    {
        lastMsg = completeMessage;
    }

    ReceivedMavlinkMessage lastMsg;
};

TEST(MavlinkReceiver, test_crc_calculation){
    Drivers drivers;

    uint8_t rawMessage[8];
    uint16_t currByte = 0;

    rawMessage[0] = 0xFE;
    rawMessage[1] = 0;
    rawMessage[2] = 1;
    rawMessage[3] = 2;
    rawMessage[4] = 3;
    rawMessage[5] = 32;

    uint16_t crc_calc;

    MavlinkReceiver::crc_init(&crc_calc);

    uint16_t crc_actual = 0xffff;

    EXPECT_EQ(crc_calc, crc_actual);

    crc_calc = MavlinkReceiver::crc_calculate(rawMessage + 1, 5);


    MavlinkReceiver::crc_accumulate(rawMessage[1], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[2], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[3], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[4], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[5], &crc_actual);

    EXPECT_EQ(crc_calc, crc_actual);

    MavlinkReceiver::crc_accumulate(185, &crc_calc);

    MavlinkReceiver::crc_accumulate(MavlinkReceiver::CRC_EXTRA[0], &crc_actual);
}

TEST(MavlinkReceiver, updateSerial_parseMessage_single_byte_empty_message)
{
    Drivers drivers;
    MavlinkReceiverTester serial(&drivers, Uart::Uart7);

    uint8_t rawMessage[8];
    uint16_t currByte = 0;

    rawMessage[0] = 0xFE;
    rawMessage[1] = 0;
    rawMessage[2] = 1;
    rawMessage[3] = 2;
    rawMessage[4] = 3;
    rawMessage[5] = 32;

    // 6 and 7 are CRC bytes, ignore the head byte
    uint16_t crc_actual = 0xffff;

    MavlinkReceiver::crc_accumulate(rawMessage[1], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[2], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[3], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[4], &crc_actual);
    MavlinkReceiver::crc_accumulate(rawMessage[5], &crc_actual);
    MavlinkReceiver::crc_accumulate(MavlinkReceiver::CRC_EXTRA[32], &crc_actual);

    // EXPECT_EQ(crc_actual,100);

    rawMessage[6] = crc_actual & 0xFF;  // low byte (rightmost)
    EXPECT_EQ(rawMessage[6], 0b01011111);

    rawMessage[7] = crc_actual >> 8;  // high byte (leftmost)
    EXPECT_EQ(rawMessage[7], 0b11011110);

    ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
        .WillByDefault(
            [&](Uart::UartPort, uint8_t *data, std::size_t length)
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

    // for (int i = 0; i < 8; i++)
    // {
    // }

    serial.updateSerial();
    EXPECT_EQ(serial.newMessage.header.headByte, 0xFE);
    serial.updateSerial();
    EXPECT_EQ(serial.newMessage.header.dataLength, 0);
    serial.updateSerial();
    EXPECT_EQ(serial.newMessage.header.seq, 1);
    serial.updateSerial();
    EXPECT_EQ(serial.newMessage.header.systemId, 2);
    serial.updateSerial();
    EXPECT_EQ(serial.newMessage.header.componentId, 3);
    serial.updateSerial();
    EXPECT_EQ(serial.newMessage.header.messageId, 32);

    serial.updateSerial();
    serial.updateSerial();

    EXPECT_EQ(serial.bytesToRead, 2);

    EXPECT_EQ(serial.tempCRC, crc_actual);

    EXPECT_EQ(serial.tempCRC, serial.newMessage.CRC16);

    EXPECT_EQ(serial.newMessage.CRC16, crc_actual);

    // EXPECT_EQ(serial.lastMsg.header.headByte, 0xFE);
    // EXPECT_EQ(serial.lastMsg.header.dataLength, 0);
    // EXPECT_EQ(serial.lastMsg.header.seq, 1);
    // EXPECT_EQ(serial.lastMsg.header.systemId, 2);
    // EXPECT_EQ(serial.lastMsg.header.componentId, 3);
    // EXPECT_EQ(serial.lastMsg.header.messageId, 32);
    // EXPECT_EQ(serial.lastMsg.CRC16, crc_actual);
}

// TEST(MavlinkReceiver, updateSerial_parseMessage_single_byte_randomData)
// {
//     Drivers drivers;
//     MavlinkReceiverTester serial(&drivers, Uart::Uart7);

//     uint8_t rawMessage[18];
//     uint16_t currByte = 0;

//     rawMessage[0] = 0xFE;
//     rawMessage[1] = 10;
//     rawMessage[2] = 2;
//     rawMessage[3] = 0;
//     rawMessage[4] = 1;
//     rawMessage[5] = 33;
//     for (uint8_t i = 0; i < 10; i++)
//     {
//         rawMessage[i + 6] = i;
//     }

//     uint16_t crc_actual = calculateCRC16(reinterpret_cast<uint8_t *>(rawMessage) + 1, 15);

//     rawMessage[16] = crc_actual & 0xFF;  // low byte (rightmost)
//     rawMessage[17] = crc_actual >> 8;    // high byte (leftmost)

//     ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
//         .WillByDefault(
//             [&](Uart::UartPort, uint8_t *data, std::size_t length)
//             {
//                 if (length == 0)
//                 {
//                     return 0;
//                 }

//                 if (currByte >= sizeof(rawMessage))
//                 {
//                     return 0;
//                 }
//                 *data = rawMessage[currByte];
//                 currByte++;
//                 return 1;
//             });

//     for (int i = 0; i < 18; i++)
//     {
//         serial.updateSerial();
//     }

//     EXPECT_EQ(serial.lastMsg.header.headByte, 0xFE);
//     EXPECT_EQ(serial.lastMsg.header.dataLength, 10);
//     EXPECT_EQ(serial.lastMsg.header.seq, 2);
//     EXPECT_EQ(serial.lastMsg.header.systemId, 0);
//     EXPECT_EQ(serial.lastMsg.header.componentId, 1);
//     EXPECT_EQ(serial.lastMsg.header.messageId, 33);

//     for (uint8_t i = 0; i < 10; i++)
//     {
//         EXPECT_EQ(serial.lastMsg.data[i], i);
//     }

//     EXPECT_EQ(serial.lastMsg.CRC16, crc_actual);
// }

// TEST(MavlinkReceiver, updateSerial_CRC16_off_by_one)
// {
//     Drivers drivers;
//     MavlinkReceiverTester serial(&drivers, Uart::Uart7);

//     EXPECT_CALL(drivers.errorController, addToErrorList)
//         .WillOnce([&](const tap::errors::SystemError &error)
//                   { EXPECT_TRUE(errorDescriptionContainsSubstr(error, "CRC16 failure")); });

//     uint8_t rawMessage[18];
//     uint16_t currByte = 0;

//     rawMessage[0] = 0xFE;
//     rawMessage[1] = 10;
//     rawMessage[2] = 3;
//     rawMessage[3] = 0;
//     rawMessage[4] = 1;
//     rawMessage[5] = 34;
//     for (uint8_t i = 0; i < 10; i++)
//     {
//         rawMessage[i + 6] = i;
//     }

//     uint16_t crc_actual = calculateCRC16(reinterpret_cast<uint8_t *>(rawMessage) + 1, 15) - 1;

//     rawMessage[16] = crc_actual & 0xFF;  // low byte (rightmost)
//     rawMessage[17] = crc_actual >> 8;    // high byte (leftmost)

//     ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
//         .WillByDefault(
//             [&](Uart::UartPort, uint8_t *data, std::size_t length)
//             {
//                 if (length == 0)
//                 {
//                     return 0;
//                 }

//                 if (currByte >= sizeof(rawMessage))
//                 {
//                     return 0;
//                 }
//                 *data = rawMessage[currByte];
//                 currByte++;
//                 return 1;
//             });

//     for (int i = 0; i < 18; i++)
//     {
//         serial.updateSerial();
//     }
// }

// TEST(MavlinkReciever, updateSerial_all_bytes_at_once)
// {
//     Drivers drivers;
//     MavlinkReceiverTester serial(&drivers, Uart::Uart7);

//     uint8_t rawMessage[18];
//     uint16_t currByte = 0;

//     rawMessage[0] = 0xFE;
//     rawMessage[1] = 10;
//     rawMessage[2] = 4;
//     rawMessage[3] = 0;
//     rawMessage[4] = 1;
//     rawMessage[5] = 35;
//     for (uint8_t i = 0; i < 10; i++)
//     {
//         rawMessage[i + 6] = i;
//     }

//     uint16_t crc_actual = calculateCRC16(reinterpret_cast<uint8_t *>(rawMessage) + 1, 15);

//     rawMessage[16] = crc_actual & 0xFF;  // low byte (rightmost)
//     rawMessage[17] = crc_actual >> 8;    // high byte (leftmost)

//     ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
//         .WillByDefault(
//             [&](Uart::UartPort, uint8_t *data, std::size_t length)
//             {
//                 int bytesRead = 0;
//                 for (std::size_t i = 0; i < length; i++)
//                 {
//                     if (currByte == sizeof(rawMessage))
//                     {
//                         return bytesRead;
//                     }
//                     else
//                     {
//                         data[i] = rawMessage[currByte];
//                         currByte++;
//                         bytesRead++;
//                     }
//                 }
//                 return bytesRead;
//             });

//     for (int i = 0; i < 3; i++)
//     {
//         serial.updateSerial();
//     }

//     EXPECT_EQ(serial.lastMsg.header.headByte, 0xFE);
//     EXPECT_EQ(serial.lastMsg.header.dataLength, 10);
//     EXPECT_EQ(serial.lastMsg.header.seq, 4);
//     EXPECT_EQ(serial.lastMsg.header.systemId, 0);
//     EXPECT_EQ(serial.lastMsg.header.componentId, 1);
//     EXPECT_EQ(serial.lastMsg.header.messageId, 35);

//     for (uint8_t i = 0; i < 10; i++)
//     {
//         EXPECT_EQ(serial.lastMsg.data[i], i);
//     }

//     EXPECT_EQ(serial.lastMsg.CRC16, crc_actual);
// }
