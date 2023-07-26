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

#include "aruwsrc/robot/drone/drone_telemetry_handler.hpp"

using namespace tap::communication::serial;
using namespace tap::arch;
using namespace testing;
using namespace tap;
using namespace tap::algorithms;
using namespace aruwsrc::communication::serial;
using namespace aruwsrc::drone;

// TEST(DroneTelemetryHandler, receive_complete_msg_positives)
// {
//     Drivers drivers;
//     DroneTelemetryHandler serial(&drivers, Uart::Uart7);

//     LocalPositionNed positionMsg = {
//         .time_boot_ms = 0,
// 		.x = 1,
// 		.y = 2,
// 		.z = 3,
// 		.vx = 4,
// 		.vy = 5,
// 		.vz = 6,
//     };

//     uint8_t rawMessage[36];
//     uint16_t currByte = 0;

//     rawMessage[0] = 0xFE;
//     rawMessage[1] = 28;
//     rawMessage[2] = 1;
//     rawMessage[3] = 0;
//     rawMessage[4] = 1;
//     rawMessage[5] = 32;

// 	memcpy(rawMessage + 6, &positionMsg, sizeof(positionMsg));

// 	uint16_t crc = calculateCRC16(reinterpret_cast<uint8_t *>(rawMessage) + 1, 33);

//     rawMessage[34] = crc & 0xFF;  // low byte (rightmost)
//     rawMessage[35] = crc >> 8;    // high byte (leftmost)

// 	ON_CALL(drivers.uart, read(Uart::Uart7, _, _))
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
	
// 	for (int i = 0; i < 36; i++)
//     {
//         serial.updateSerial();
//     }

// 	EXPECT_EQ(serial.localPositionNed.time_boot_ms, positionMsg.time_boot_ms);
// 	EXPECT_EQ(serial.localPositionNed.x, positionMsg.x);
// 	EXPECT_EQ(serial.localPositionNed.y, positionMsg.y);
// 	EXPECT_EQ(serial.localPositionNed.z, positionMsg.z);
// 	EXPECT_EQ(serial.localPositionNed.vx, positionMsg.vx);
// 	EXPECT_EQ(serial.localPositionNed.vy, positionMsg.vy);
// 	EXPECT_EQ(serial.localPositionNed.vz, positionMsg.vz);




// }
