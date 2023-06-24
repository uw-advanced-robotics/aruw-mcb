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
            CRC16 = crc_calculate(
                reinterpret_cast<uint8_t *>(this) + 1,
                5);                                                  // Calculate for header first
            crc_accumulate_buffer(&CRC16, data, header.dataLength);  // Then calculate for data
            crc_accumulate(CRC_EXTRA[header.messageId], &CRC16);     // Then calculate for crc_extra
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

#ifndef ENV_UNIT_TESTS
protected:
#endif
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
    constexpr static int UART_BAUDRATE = 115'200;

    tap::Drivers *drivers;

    /// The serial port you are connected to.
    Uart::UartPort port;

    int bytesToRead;
    uint64_t headBytesCorrect = 0;
    uint16_t settingHeader = 0;
    uint16_t failedCRC = 0;
    uint16_t readTooMuch = 0;
    uint16_t gotaThirtyTwoMessageID = 0;
    uint16_t readAFullMessage = 0;
    uint16_t reading = 0;
    uint16_t readFromUart = 0;

    uint32_t datathingy[255];

    /**
     * Here's a bunch of code blatantly copied over from the mavlink page
     * https://github.com/mavlink/c_library_v1/blob/master/checksum.h to calculate the checksum
     */

    static constexpr uint8_t CRC_EXTRA[] = {
        50,  124, 137, 0,   237, 217, 104, 119, 117, 0,   0,   89,  0,   0,   0,   0,   0,   0,
        0,   0,   214, 159, 220, 168, 24,  23,  170, 144, 67,  115, 39,  246, 185, 104, 237, 244,
        222, 212, 9,   254, 230, 28,  28,  132, 221, 232, 11,  153, 41,  39,  78,  196, 0,   0,
        15,  3,   0,   0,   0,   0,   0,   167, 183, 119, 191, 118, 148, 21,  0,   243, 124, 0,
        0,   38,  20,  158, 152, 143, 0,   0,   14,  106, 49,  22,  143, 140, 5,   150, 0,   231,
        183, 63,  54,  47,  0,   0,   0,   0,   0,   0,   175, 102, 158, 208, 56,  93,  138, 108,
        32,  185, 84,  34,  174, 124, 237, 4,   76,  128, 56,  116, 134, 237, 203, 250, 87,  203,
        220, 25,  226, 46,  29,  223, 85,  6,   229, 203, 1,   195, 109, 168, 181, 47,  72,  131,
        127, 0,   103, 154, 178, 200, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        189, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   36,  0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   208, 0,   0,   0,   0,   163, 105, 151, 35,
        150, 179, 0,   0,   0,   0,   0,   90,  104, 85,  95,  130, 184, 81,  8,   204, 49,  170,
        44,  83,  46,  0};

    static constexpr uint16_t X25_INIT_CRC = 0xffff;

    static inline uint16_t calculateCRC(ReceivedMavlinkMessage msg)
    {
        uint16_t CRC16 = crc_calculate(
            reinterpret_cast<uint8_t *>(&msg) + 1,
            5);  // Calculate for header first
        crc_accumulate_buffer(
            &CRC16,
            reinterpret_cast<const char *>(msg.data),
            msg.header.dataLength);                           // Then calculate for data
        crc_accumulate(CRC_EXTRA[msg.header.messageId], &CRC16);  // Then calculate for crc_extra
        return CRC16;
    }

    /**
     * @brief Accumulate the MCRF4XX CRC16 by adding one char at a time.
     *
     * The checksum function adds the hash of one char at a time to the
     * 16 bit checksum (uint16_t).
     *
     * @param data new char to hash
     * @param crcAccum the already accumulated checksum
     **/
    static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
    {
        /*Accumulate one byte of data into the CRC*/
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum & 0xff);
        tmp ^= (tmp << 4);
        *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }

    /**
     * @brief Initialize the buffer for the MCRF4XX CRC16
     *
     * @param crcAccum the 16 bit MCRF4XX CRC16
     */
    static inline void crc_init(uint16_t *crcAccum) { *crcAccum = X25_INIT_CRC; }

    /**
     * @brief Calculates the MCRF4XX CRC16 checksum on a byte buffer
     *
     * @param  pBuffer buffer containing the byte array to hash
     * @param  length  length of the byte array
     * @return the checksum over the buffer bytes
     **/
    static inline uint16_t crc_calculate(const uint8_t *pBuffer, uint16_t length)
    {
        uint16_t crcTmp;
        crc_init(&crcTmp);
        while (length--)
        {
            crc_accumulate(*pBuffer++, &crcTmp);
        }
        return crcTmp;
    }

    /**
     * @brief Accumulate the MCRF4XX CRC16 CRC by adding an array of bytes
     *
     * The checksum function adds the hash of one char at a time to the
     * 16 bit checksum (uint16_t).
     *
     * @param data new bytes to hash
     * @param crcAccum the already accumulated checksum
     **/
    static inline void crc_accumulate_buffer(
        uint16_t *crcAccum,
        const char *pBuffer,
        uint16_t length)
    {
        const uint8_t *p = (const uint8_t *)pBuffer;
        while (length--)
        {
            crc_accumulate(*p++, crcAccum);
        }
    }
};

}  // namespace aruwsrc::communication::serial

#endif
