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

#ifndef MTF_01_HPP_
#define MTF_01_HPP_

#include "tap/algorithms/transforms/vector.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::communication::serial
{

struct mtf01
{
    struct MicrolinkMessage
    {
        uint8_t header;
        uint8_t device_id;
        uint8_t system_id;
        uint8_t msg_id;
        uint8_t seq;
        uint8_t data_length;
        struct payload
        {
            uint32_t timestamp_ms;
            uint32_t distance_mm;  // Minimum value of 1, 0 for invalid
            uint8_t distance_strength;
            uint8_t distance_precision;  // Smaller value indicates higher accuracy
            bool distance_valid;
            uint8_t reserved;
            uint16_t optical_flow_x;  // Speed (cm/s) = flow_vel * distance
            uint16_t optical_flow_y;
            uint8_t optical_flow_quality;  // Bigger values indicates higher quality
            bool optical_flow_valid;
            uint16_t reserved2;
        };
        payload payload;
        uint8_t checksum;
    } modm_packed;

    static constexpr uint8_t NUM_BYTES_MESSAGE = sizeof(MicrolinkMessage);

    enum class ExpectedMessage : uint8_t
    {
        HEADER = 0xEF,
        DEVICE_ID = 0x0F,
        SYSTEM_ID = 0x00,
        MSG_ID = 0x51,
        PAYLOAD_LENGTH = 0x14
    };

    // Checks message headers and CRC
    static bool validateMessage(const MicrolinkMessage &msg)
    {
        if (msg.header != static_cast<uint8_t>(ExpectedMessage::HEADER))
        {
            return false;
        }
        if (msg.device_id != static_cast<uint8_t>(ExpectedMessage::DEVICE_ID))
        {
            return false;
        }
        if (msg.system_id != static_cast<uint8_t>(ExpectedMessage::SYSTEM_ID))
        {
            return false;
        }
        if (msg.msg_id != static_cast<uint8_t>(ExpectedMessage::MSG_ID))
        {
            return false;
        }
        if (msg.data_length != static_cast<uint8_t>(ExpectedMessage::PAYLOAD_LENGTH))
        {
            return false;
        }

        // Checksum calculation
        int checksum = 0;
        for (int i = 0; i < NUM_BYTES_MESSAGE - 1; i++)
        {
            checksum += ((uint8_t *)&msg)[i];
        }
        if (checksum != msg.checksum)
        {
            return false;
        }

        return true;
    }

    // Returns translation in meters per second
    static tap::algorithms::transforms::Vector getVelocity(const MicrolinkMessage &msg)
    {
        float distance_m = msg.payload.distance_mm / 1000.0;
        float x =
            msg.payload.optical_flow_x * distance_m / 100.0;  // Conversion to cm/s and then to m/s
        float y = msg.payload.optical_flow_y * distance_m / 100.0;
        return tap::algorithms::transforms::Vector(x, y, 0);
    }
};

/**
 * Class to interface with the MTF01 Optical flow and distance sensor.
 * This class is being used to parse the Microlink protocol as defined on this page:
 * https://micoair.com/optical_range_sensor_mtf-01/
 */
class MTF01 : public mtf01
{
public:
    MTF01(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort port);

    static constexpr int BAUDRATE = 115'200;

    void initialize();

    void read();

    inline tap::algorithms::transforms::Vector getRelativeVelocity()
    {
        return mtf01::getVelocity(processedMessage);
    }

private:
    tap::Drivers *drivers;
    const tap::communication::serial::Uart::UartPort port;

    struct MicrolinkMessage currentMessage;
    struct MicrolinkMessage processedMessage;
};

}  // namespace aruwsrc::commuinication::serial

#endif // MTF_01_HPP_
