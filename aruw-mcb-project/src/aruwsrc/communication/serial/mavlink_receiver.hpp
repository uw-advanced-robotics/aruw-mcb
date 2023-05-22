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

#ifndef MAVLINK_RECEIVER_HPP_
#define MAVLINK_RECEIVER_HPP_

#include "tap/algorithms/crc.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::serial
{
/**
 * A serial handler that implements a specific protocol to be used for
 * communicating with mavlink devices, such as the cube orange on drone.
 * This is a message handler for the mavlink 1 message protocol.
 *
 * This is like a blatant copy of DJISerial.hpp, but with adjusted file format.
 * Lookie here: https://mavlink.io/en/guide/serialization.html
 *
 * Extend this class and implement messageReceiveCallback if you
 * want to use this mavlink protocol on a serial line.
 *
 * Note, the CRC does not include the header byte in computation.
 *
 * Structure of a Mavlink Message:
 * \rst
 * +-----------------+------------------------------------------------------------+
 * | Byte Number     | Byte Description                                           |
 * +=================+============================================================+
 * | Frame Header                                                                 |
 * +-----------------+------------------------------------------------------------+
 * | 0               | Packet Start Marker (0xFE)                                 |
 * +-----------------+------------------------------------------------------------+
 * | 1               | Payload length 			                                  |
 * +-----------------+------------------------------------------------------------+
 * | 2               | Packet sequence number                                     |
 * +-----------------+------------------------------------------------------------+
 * | 3               | System ID (ID of the system sending the message)           |
 * +-----------------+------------------------------------------------------------+
 * | 4               | Component ID                                               |
 * +-----------------+------------------------------------------------------------+
 * | 5               | Message ID                                                 |
 * +-----------------+------------------------------------------------------------+
 * | Body - Data Length bytes                                                     |
 * +-----------------+------------------------------------------------------------+
 * | Message CRC                                                                  |
 * +-----------------+------------------------------------------------------------+
 * | 7 + Data Length | CRC16                                                      |
 * +-----------------+------------------------------------------------------------+
 * | 8 + Data Length | CRC16                                                      |
 * +-----------------+------------------------------------------------------------+
 * \endrst
 */
class MavlinkReceiver
{
public:
    /**
     * The mavlink message's frame header.
     */
    struct FrameHeader
    {
        uint8_t headByte;
        uint8_t dataLength;
        uint8_t seq;
        uint8_t systemId;
        uint8_t componentId;
        uint8_t messageId;
    } modm_packed;

    /**
     * A container for storing and sending message over serial.
     */
    template <int DATA_SIZE>
    struct MavlinkMessage
    {
        /**
         * Constructs a MavlinkMessage. In doing so this constructor configures the message header.
         *
         * @param[in] seq Message sequence number, an optional parameter.
         */
        explicit MavlinkMessage(
            uint8_t seq = 0,
            uint8_t systemId = 0,
            uint8_t componentId = 0,
            uint8_t messageId = 0)
        {
            header.headByte = SERIAL_HEAD_BYTE;
            header.dataLength = sizeof(data);
            header.seq = seq;
            header.systemId = systemId;
            header.componentId = componentId;
            header.messageId = messageId;
        }

        /**
         * Sets the CRC16 value in the struct. This should be called after writing data to the
         * message struct.
         */
        void setCRC16()
        {
            CRC16 = tap::algorithms::calculateCRC16(
                reinterpret_cast<uint8_t *>(this)[1],  // We don't want to include the head byte
                sizeof(*this) - 3);
        }

        FrameHeader header;
        uint8_t data[DATA_SIZE];
        uint16_t CRC16;
    } modm_packed;

    static const uint8_t SERIAL_RX_BUFF_SIZE =
        255;  // This is 256 in DJI serial but like max is 255
    static const uint8_t SERIAL_HEAD_BYTE = 0xFE;

    using ReceivedMavlinkMessage = MavlinkMessage<SERIAL_RX_BUFF_SIZE>;

    /**
     * Construct a Serial object.
     *
     * @param[in] port serial port to work on.
     * @param[in] isRxCRCEnforcementEnabled `true` to enable Rx CRC Enforcement.
     */
    MavlinkReceiver(tap::Drivers *drivers, Uart::UartPort port);
    DISALLOW_COPY_AND_ASSIGN(MavlinkReceiver)
    mockable ~MavlinkReceiver() = default;

    /**
     * Initialize serial. In particular, initializes the hardware serial
     * specified upon construction.
     *
     * @note currently, only uart ports 1, 2, 3, and 6 are enabled. Be sure
     *      to add a serial port to `uart.hpp` if you want to use the serial.
     *      Also, if you add a new uart port to be generated in the `project.xml`
     *      file, you should add it to both the `Uart` class and this function.
     * @see `Uart`
     */
    mockable void initialize();

    /**
     * Receive messages. Call periodically in order to receive all
     * incoming messages.
     */
    mockable void updateSerial();

    /**
     * Called when a complete message is received. A derived class must
     * implement this in order to handle incoming messages properly.
     *
     * @param[in] completeMessage a reference to the full message that has
     *      just been received by this class.
     */
    virtual void messageReceiveCallback(const ReceivedMavlinkMessage &completeMessage) = 0;

private:
    enum SerialRxState
    {
        SERIAL_HEADER_SEARCH,  /// A header byte has not yet been received.
        PROCESS_FRAME_HEADER,  /// A header is received and the frame header is being processed.
        PROCESS_FRAME_DATA     /// The data is being processed.
    };

    

    /// stuff for RX, buffers to store parts of the header, state machine.
    SerialRxState mavlinkSerialRxState;

    /// Message in middle of being constructed.
    ReceivedMavlinkMessage newMessage;

    /// Most recent complete message.
    ReceivedMavlinkMessage mostRecentMessage;

    /**
     * The current zero indexed byte that is being read from the `Uart` class. An absolute index
     * into the `newMessage` object. `newMessage` reinterpreted as a uint8_t array and elements read
     * from serial are placed into the message.
     */
    uint16_t frameCurrReadByte;

    // This is default on a cube orange as per:
    // https://docs.cubepilot.org/user-guides/autopilot/the-cube-user-manual
    constexpr static int UART_BAUDRATE = 57'600;

protected:
    tap::Drivers *drivers;
    
    /// The serial port you are connected to.
    Uart::UartPort port;

    int bytesToRead;
    uint16_t headBytesCorrect = 0;
    uint16_t settingHeader = 0;
    uint16_t readHeaderAndDataLengthToLong = 0;
    uint16_t failedCRC = 0;
    uint16_t readTooMuch = 0;
    uint16_t gotaThirtyTwoMEssageID = 0;
};

}  // namespace aruwsrc::communication::serial

#endif
