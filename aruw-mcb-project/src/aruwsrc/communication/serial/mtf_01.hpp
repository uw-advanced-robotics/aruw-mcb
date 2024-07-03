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

#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/math/geometry/vector.hpp"

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
            uint32_t time_ms;              /// System time in ms
            uint32_t distance;             /// distance(mm), 0 Indicates unavailable
            uint8_t distance_strength;     /// signal strength
            uint8_t distance_precision;    /// distance precision, smaller values better
            uint8_t distance_status;       /// distance status
            uint8_t reserved1;             /// reserved
            int16_t optical_flow_vel_x;    /// optical flow velocity in x, cm/s when mutiplied by
                                           /// distance in m
            int16_t optical_flow_vel_y;    /// optical flow velocity in y
            uint8_t optical_flow_quality;  /// optical flow quality, bigger values better
            uint8_t optical_flow_status;   /// optical flow status
            uint16_t reserved2;            /// reserved
        };
        payload payload;
        uint8_t checksum;
        uint8_t status;  // These values are defined in the protocol, but are documented in the
                         // MicoLink documentation
        uint8_t payload_count;
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

    int headerAndStuffWasWrong = 0;
    int failedCRC = 0;

protected:
    // Checks message headers and CRC
    bool validateMessage(const MicrolinkMessage &msg)
    {
        if (msg.header != static_cast<uint8_t>(ExpectedMessage::HEADER))
        {
            headerAndStuffWasWrong++;
            return false;
        }
        if (msg.device_id != static_cast<uint8_t>(ExpectedMessage::DEVICE_ID))
        {
            headerAndStuffWasWrong++;
            return false;
        }
        if (msg.system_id != static_cast<uint8_t>(ExpectedMessage::SYSTEM_ID))
        {
            headerAndStuffWasWrong++;
            return false;
        }
        if (msg.msg_id != static_cast<uint8_t>(ExpectedMessage::MSG_ID))
        {
            headerAndStuffWasWrong++;
            return false;
        }
        if (msg.data_length != static_cast<uint8_t>(ExpectedMessage::PAYLOAD_LENGTH))
        {
            headerAndStuffWasWrong++;
            return false;
        }

        // Checksum calculation
        uint8_t checksum = 0;
        int length = NUM_BYTES_MESSAGE - 3;  // Exclude checksum, status, and payload_count
        for (int i = 0; i < length; i++)
        {
            checksum += reinterpret_cast<const uint8_t *>(&msg)[i];
        }

        if (checksum != msg.checksum)
        {
            failedCRC++;
            return false;
        }

        return true;
    }

    // Returns translation in meters per second
    static modm::Vector2f getVelocity(const MicrolinkMessage &msg)
    {
        float distance_m = msg.payload.distance / 1000.0f;  // Conversion to meters from mm
        // Conversion to cm/s and then to m/s
        float x = msg.payload.optical_flow_vel_x * distance_m / 100.0;
        float y = msg.payload.optical_flow_vel_y * distance_m / 100.0;
        return modm::Vector2f(x, y);
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

    inline modm::Vector2f getRelativeVelocity() { return mtf01::getVelocity(processedMessage); }

    inline uint8_t getFlowQuality() { return processedMessage.payload.optical_flow_quality; }

private:
    tap::Drivers *drivers;
    const tap::communication::serial::Uart::UartPort port;

    struct MicrolinkMessage currentMessage;
    struct MicrolinkMessage processedMessage;
    int currentFrameIndex = 0;

    enum ParsingState
    {
        SERIAL_HEADER_SEARCH,  /// A header byte has not yet been received.
        PROCESS_FRAME_HEADER   /// A header is received and the message is being processed.
    };

    ParsingState state = SERIAL_HEADER_SEARCH;
};

}  // namespace aruwsrc::communication::serial

#endif  // MTF_01_HPP_
