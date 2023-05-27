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
    MavlinkReceiverTester(Drivers *drivers, Uart::UartPort port)
        : MavlinkReceiver(drivers, port)
    {
    }

    void messageReceiveCallback(const ReceivedMavlinkMessage &completeMessage) override
    {
        lastMsg = completeMessage;
    }

    ReceivedMavlinkMessage lastMsg;
};

TEST(MavlinkReceiver, updateSerial_parseMessage_single_byte){
	Drivers drivers;
	MavlinkReceiverTester serial(&drivers, Uart::Uart7);

	uint8_t rawMessage[8];
	uint16_t currByte = 0;

	rawMessage[0] = 0xFE;
	rawMessage[1] = 0;
	rawMessage[2] = 1;
	rawMessage[3] = 2;
	rawMessage[4] = 3;
	rawMessage[5] = 33;
	// 6 and 7 are CRC bytes, ignore the head byte
	rawMessage[6] = calculateCRC16(&rawMessage[1], 5);
	rawMessage[7] = calculateCRC16(&rawMessage[1], 5) >> 8;
	
	ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
        .WillByDefault([&](Uart::UartPort, uint8_t *data, std::size_t length) {
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

	for(int i = 0; i < 8; i++){
		serial.updateSerial();
	}

	EXPECT_EQ(serial.lastMsg.header.headByte, 0xFE);
	EXPECT_EQ(serial.lastMsg.header.dataLength, 0);
	EXPECT_EQ(serial.lastMsg.header.seq, 1);
	EXPECT_EQ(serial.lastMsg.header.systemId, 2);
	EXPECT_EQ(serial.lastMsg.header.componentId, 3);
	EXPECT_EQ(serial.lastMsg.header.messageId, 33);



}


