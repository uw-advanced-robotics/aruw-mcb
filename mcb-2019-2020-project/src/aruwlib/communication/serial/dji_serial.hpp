#ifndef __serial_h_
#define __serial_h_

#include <cstdint>

#include <modm/processing.hpp>

#include "aruwlib/algorithms/crc.hpp"
#include "aruwlib/errors/create_errors.hpp"
#include "aruwlib/errors/system_error.hpp"

#include "uart.hpp"

namespace aruwlib
{
namespace serial
{
/**
 * A container for storing TX and RX messages.
 */
struct SerialMessage
{
    static constexpr uint16_t SERIAL_RX_BUFF_SIZE = 256;
    uint8_t headByte;  ///< Use SERIAL_HEAD_BYTE.
    uint16_t length;   ///< Must be less than SERIAL_RX_BUFF_SIZE or SERIAL_TX_BUFF_SIZE.
    uint16_t type;     ///< The type is specified and interpreted by a derived class.
    uint8_t data[SERIAL_RX_BUFF_SIZE];
    modm::Timestamp messageTimestamp;  ///< The timestamp is in milliseconds.
    uint8_t sequenceNumber;  ///< A derived class may increment this for debugging purposes.
};

/**
 * A serial handler that implements a specific protocol to be used for
 * communicating with the referee system. Also used for our personal
 * communication with the xavier.
 *
 * Extend this class and implement messageReceiveCallback if you
 * want to use this serial protocol on a serial line.
 *
 * Structure of a Serial Message:
 * \rst
 * +-----------------+------------------------------------------------------------+
 * | Byte Number     | Byte Description                                           |
 * +=================+============================================================+
 * | Frame Header                                                                 |
 * +-----------------+------------------------------------------------------------+
 * | 0               | Frame Head Byte (0xA5)                                     |
 * +-----------------+------------------------------------------------------------+
 * | 1               | Frame Data Length, LSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 2               | Frame Data Length, MSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 3               | Frame Sequence Number                                      |
 * +-----------------+------------------------------------------------------------+
 * | 4               | CRC8 of the frame, (bytes 0 - 3)                           |
 * +-----------------+------------------------------------------------------------+
 * | 5               | Message Type, LSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | 6               | Message Type, MSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | Body - Data Length bytes                                                     |
 * +-----------------+------------------------------------------------------------+
 * | Message CRC                                                                  |
 * +-----------------+------------------------------------------------------------+
 * | 7 + Data Length | CRC16 of header and frame, LSB (bytes 0 - 6 + Data Length) |
 * +-----------------+------------------------------------------------------------+
 * | 8 + Data Length | CRC16 of header and frame, MSB                             |
 * +-----------------+------------------------------------------------------------+
 * \endrst
 */
template <typename Drivers> class DJISerial
{
private:
    static constexpr uint16_t SERIAL_TX_BUFF_SIZE = 256;
    static constexpr uint16_t SERIAL_HEAD_BYTE = 0xA5;
    static constexpr uint8_t FRAME_DATA_LENGTH_OFFSET = 1;
    static constexpr uint8_t FRAME_SEQUENCENUM_OFFSET = 3;
    static constexpr uint8_t FRAME_CRC8_OFFSET = 4;
    static constexpr uint8_t FRAME_HEADER_LENGTH = 7;
    static constexpr uint8_t FRAME_TYPE_OFFSET = 5;
    static constexpr uint8_t FRAME_CRC16_LENGTH = 2;

public:
    /**
     * Construct a Serial object.
     *
     * @param[in] port serial port to work on.
     * @param[in] isRxCRCEnforcementEnabled if to enable Rx CRC Enforcement.
     */
    DJISerial(Uart::UartPort port, bool isRxCRCEnforcementEnabled)
        : port(port),
          djiSerialRxState(SERIAL_HEADER_SEARCH),
          frameCurrReadByte(0),
          frameHeader(),
          rxCRCEnforcementEnabled(isRxCRCEnforcementEnabled),
          txBuffer()
    {
        txMessage.length = 0;
        newMessage.length = 0;
        mostRecentMessage.length = 0;
    }

    /**
     * Initialize serial. In particular, initializes the hardware serial
     * specified upon construction.
     *
     * @note currently, only uart ports 1, 2, and 6 are enabled. Be sure
     *      to add a serial port to `uart.hpp` if you want to use the serial.
     *      Also, if you add a new uart port to be generated in the `project.xml`
     *      file, you should add it to both the `Uart` class and this function.
     * @see `Uart`
     */
    void initialize()
    {
        switch (this->port)
        {
            case Uart::UartPort::Uart1:
                Drivers::uart.template init<Uart::UartPort::Uart1, 115200>();
                break;
            case Uart::UartPort::Uart2:
                Drivers::uart.template init<Uart::UartPort::Uart2, 115200>();
                break;
            case Uart::UartPort::Uart6:
                Drivers::uart.template init<Uart::UartPort::Uart6, 115200>();
                break;
            default:
                break;
        }
    }

    /**
     * Receive messages. Call periodically in order to receive all
     * incoming messages.
     *
     * @note tested with a delay of 10 microseconds with referee system. The
     *      longer the timeout the more likely a message failure may occur.
     */
    void updateSerial()
    {
        switch (djiSerialRxState)
        {
            case SERIAL_HEADER_SEARCH:
            {
                // keep scanning for the head byte as long as you are here and have not yet found
                // it.
                uint8_t serialHeadCheck = 0;
                while (djiSerialRxState == SERIAL_HEADER_SEARCH && read(&serialHeadCheck, 1))
                {
                    // we found it, store the head byte
                    if (serialHeadCheck == SERIAL_HEAD_BYTE)
                    {
                        frameHeader[0] = SERIAL_HEAD_BYTE;
                        newMessage.headByte = SERIAL_HEAD_BYTE;
                        djiSerialRxState = PROCESS_FRAME_HEADER;
                    }
                }

                break;
            }
            case PROCESS_FRAME_HEADER:  // the frame header consists of the length, type, and CRC8
            {
                // Read from the buffer. Keep track of the index in the frameHeader array using the
                // frameCurrReadByte. +1 at beginning and -1 on the end since the serial head
                // byte is part of the frame but has already been processed.
                frameCurrReadByte += read(
                    frameHeader + frameCurrReadByte + 1,
                    FRAME_HEADER_LENGTH - frameCurrReadByte - 1);

                // We have the complete message header in the frameHeader buffer
                if (frameCurrReadByte == FRAME_HEADER_LENGTH - 1)
                {
                    frameCurrReadByte = 0;

                    // process length
                    newMessage.length = (frameHeader[FRAME_DATA_LENGTH_OFFSET + 1] << 8) |
                                        frameHeader[FRAME_DATA_LENGTH_OFFSET];
                    // process sequence number (counter)
                    newMessage.sequenceNumber = frameHeader[FRAME_SEQUENCENUM_OFFSET];
                    newMessage.type =
                        frameHeader[FRAME_TYPE_OFFSET + 1] << 8 | frameHeader[FRAME_TYPE_OFFSET];

                    if (newMessage.length == 0 ||
                        newMessage.length >= SerialMessage::SERIAL_RX_BUFF_SIZE -
                                                 (FRAME_HEADER_LENGTH + FRAME_CRC16_LENGTH))
                    {
                        djiSerialRxState = SERIAL_HEADER_SEARCH;
                        RAISE_ERROR(
                            "invalid message length received",
                            aruwlib::errors::Location::DJI_SERIAL,
                            aruwlib::errors::ErrorType::INVALID_MESSAGE_LENGTH);
                        return;
                    }

                    // check crc8 on header
                    if (rxCRCEnforcementEnabled)
                    {
                        uint8_t CRC8 = frameHeader[FRAME_CRC8_OFFSET];
                        // don't look at crc8 or frame type when calculating crc8
                        if (!verifyCRC8(frameHeader, FRAME_HEADER_LENGTH - 3, CRC8))
                        {
                            djiSerialRxState = SERIAL_HEADER_SEARCH;
                            RAISE_ERROR(
                                "CRC8 failure",
                                aruwlib::errors::Location::DJI_SERIAL,
                                aruwlib::errors::ErrorType::CRC_FAILURE);
                            return;
                        }
                    }

                    // move on to processing message body
                    djiSerialRxState = PROCESS_FRAME_DATA;
                }
                break;
            }
            case PROCESS_FRAME_DATA:  // read bulk of message
            {
                // add on extra 2 bytes for crc enforcement, and read bytes until
                // the length has been reached
                if (rxCRCEnforcementEnabled)
                {
                    frameCurrReadByte += read(
                        newMessage.data + frameCurrReadByte,
                        newMessage.length + 2 - frameCurrReadByte);
                }
                else
                {
                    frameCurrReadByte += read(
                        newMessage.data + frameCurrReadByte,
                        newMessage.length - frameCurrReadByte);
                }

                if ((frameCurrReadByte == newMessage.length && !rxCRCEnforcementEnabled) ||
                    (frameCurrReadByte == newMessage.length + 2 && rxCRCEnforcementEnabled))
                {
                    frameCurrReadByte = 0;
                    if (rxCRCEnforcementEnabled)
                    {
                        uint8_t *crc16CheckData =
                            new uint8_t[FRAME_HEADER_LENGTH + newMessage.length]();
                        memcpy(crc16CheckData, frameHeader, FRAME_HEADER_LENGTH);
                        memcpy(
                            crc16CheckData + FRAME_HEADER_LENGTH,
                            newMessage.data,
                            newMessage.length);

                        uint16_t CRC16 = (newMessage.data[newMessage.length + 1] << 8) |
                                         newMessage.data[newMessage.length];
                        if (!verifyCRC16(
                                crc16CheckData,
                                FRAME_HEADER_LENGTH + newMessage.length,
                                CRC16))
                        {
                            delete[] crc16CheckData;
                            djiSerialRxState = SERIAL_HEADER_SEARCH;
                            RAISE_ERROR(
                                "CRC16 failure",
                                aruwlib::errors::Location::DJI_SERIAL,
                                aruwlib::errors::ErrorType::CRC_FAILURE);
                            return;
                        }
                        delete[] crc16CheckData;
                    }

                    // update the time and copy over the message to the most recent message
                    newMessage.messageTimestamp = modm::Clock::now();

                    mostRecentMessage = newMessage;

                    messageReceiveCallback(mostRecentMessage);

                    djiSerialRxState = SERIAL_HEADER_SEARCH;
                }
                else if (
                    (frameCurrReadByte > newMessage.length && !rxCRCEnforcementEnabled) ||
                    (frameCurrReadByte > newMessage.length + 2 && rxCRCEnforcementEnabled))
                {
                    frameCurrReadByte = 0;
                    RAISE_ERROR(
                        "Invalid message length",
                        aruwlib::errors::Location::DJI_SERIAL,
                        aruwlib::errors::ErrorType::INVALID_MESSAGE_LENGTH);
                    djiSerialRxState = SERIAL_HEADER_SEARCH;
                }
                break;
            }
        }
    }

    /**
     * Called when a complete message is received. A derived class must
     * implement this in order to handle incoming messages properly.
     *
     * @param[in] completeMessage a reference to the full message that has
     *      just been received by this class.
     */
    virtual void messageReceiveCallback(const SerialMessage &completeMessage) = 0;

private:
    // RX related information.

    enum SerialRxState
    {
        SERIAL_HEADER_SEARCH,  ///< A header byte has not yet been received.
        PROCESS_FRAME_HEADER,  ///< A header is received and the frame header is being processed.
        PROCESS_FRAME_DATA     ///< The data is being processed.
    };

    ///< The serial port you are connected to.
    Uart::UartPort port;

    ///< stuff for RX, buffers to store parts of the header, state machine.
    SerialRxState djiSerialRxState;

    ///< Message in middle of being constructed.
    SerialMessage newMessage;

    ///< Most recent complete message.
    SerialMessage mostRecentMessage;
    /**
     * The current zero indexed byte that is being read from the `Uart` class.
     * Reset after the header is read (so increments from 0 to the header length
     * then reset to 0 and increments again from 0 to the message length, or
     * message length + 2 if crc enforcement is enabled).
     */
    uint16_t frameCurrReadByte;
    /**
     * Stores the incoming header. After the frame is received, it is transferred to
     * `newMessage`.
     */
    uint8_t frameHeader[FRAME_HEADER_LENGTH];

    ///< Whether or not to handle electrical noise.
    bool rxCRCEnforcementEnabled;

    // TX related information.

    ///< TX buffer.
    uint8_t txBuffer[SERIAL_TX_BUFF_SIZE];

    /**
     * Calculate CRC16 of given array and compare against expectedCRC16.
     *
     * @param[in] data array to calculate CRC16.
     * @param[in] length length of array to check.
     * @param[in] expectedCRC16 expected CRC16.
     * @return if the calculated CRC16 matches CRC16 given.
     */
    bool verifyCRC16(uint8_t *data, uint32_t length, uint16_t expectedCRC16)
    {
        uint16_t actualCRC16 = 0;
        if (data == nullptr)
        {
            return false;
        }
        actualCRC16 = algorithms::calculateCRC16(data, length);
        return actualCRC16 == expectedCRC16;
    }

    /**
     * Calculate CRC8 of given array and compare against expectedCRC8.
     *
     * @param[in] data array to calculate CRC8.
     * @param[in] length length of array to check.
     * @param[in] expectedCRC8 expected CRC8.
     * @return if the calculated CRC8 matches CRC8 given.
     */
    bool verifyCRC8(uint8_t *data, uint32_t length, uint8_t expectedCRC8)
    {
        uint8_t actualCRC8 = 0;
        if (data == nullptr)
        {
            return false;
        }
        actualCRC8 = algorithms::calculateCRC8(data, length);
        return actualCRC8 == expectedCRC8;
    }

    uint32_t read(uint8_t *data, uint16_t length)
    {
        return Drivers::uart.read(this->port, data, length);
    }

    uint32_t write(const uint8_t *data, uint16_t length)
    {
        if (Drivers::uart.isWriteFinished(this->port))
        {
            return Drivers::uart.write(this->port, data, length);
        }
        else
        {
            return 0;
        }
    }

protected:
    /**
     * Subclasses can access the message that this class sends as to allow
     * for modification.
     */
    SerialMessage txMessage;

    /**
     * Send a Message. This constructs a message from the `txMessage`,
     * which is protected and should be manipulated by a derived class.
     *
     * @return true if succeed, false if failed.
     */
    bool send()
    {
        txBuffer[0] = SERIAL_HEAD_BYTE;
        txBuffer[FRAME_DATA_LENGTH_OFFSET] = txMessage.length & 0xFF;
        txBuffer[FRAME_DATA_LENGTH_OFFSET + 1] = (txMessage.length >> 8) & 0xFF;
        txBuffer[FRAME_SEQUENCENUM_OFFSET] = txMessage.sequenceNumber;
        txBuffer[FRAME_CRC8_OFFSET] = algorithms::calculateCRC8(txBuffer, 4);
        txBuffer[FRAME_TYPE_OFFSET] = (txMessage.type) & 0xFF;
        txBuffer[FRAME_TYPE_OFFSET + 1] = (txMessage.type >> 8) & 0xFF;

        // we can't send, trying to send too much
        if (FRAME_HEADER_LENGTH + txMessage.length + FRAME_CRC16_LENGTH >= SERIAL_TX_BUFF_SIZE)
        {
            RAISE_ERROR(
                "dji serial attempting to send greater than SERIAL_TX_BUFF_SIZE bytes",
                aruwlib::errors::Location::DJI_SERIAL,
                aruwlib::errors::ErrorType::MESSAGE_LENGTH_OVERFLOW);
            return false;
        }

        memcpy(txBuffer + FRAME_HEADER_LENGTH, txMessage.data, txMessage.length);

        // add crc16
        uint16_t CRC16Val =
            algorithms::calculateCRC16(txBuffer, FRAME_HEADER_LENGTH + txMessage.length);
        txBuffer[FRAME_HEADER_LENGTH + txMessage.length] = CRC16Val;
        txBuffer[FRAME_HEADER_LENGTH + txMessage.length + 1] = CRC16Val >> 8;

        uint32_t totalSize = FRAME_HEADER_LENGTH + txMessage.length + FRAME_CRC16_LENGTH;
        uint32_t messageLengthSent = this->write(txBuffer, totalSize);
        if (messageLengthSent != totalSize)
        {
            return false;
            // the message did not completely send
            RAISE_ERROR(
                "the message did not completely send",
                aruwlib::errors::Location::DJI_SERIAL,
                aruwlib::errors::ErrorType::INVALID_MESSAGE_LENGTH);
        }
        txMessage.messageTimestamp = modm::Clock::now();
        return true;
    }
};

}  // namespace serial

}  // namespace aruwlib

#endif
