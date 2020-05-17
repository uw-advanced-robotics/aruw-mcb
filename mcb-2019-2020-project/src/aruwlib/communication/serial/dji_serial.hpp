#ifndef __serial_h_
#define __serial_h_

#include <modm/processing.hpp>
#include "aruwlib/rm-dev-board-a/board.hpp"
#include "aruwlib/communication/serial/uart.hpp"

namespace aruwlib
{

namespace serial
{

/**
 * A serial handler that implements a specific protocol to be used for
 * communicating with the referee system. Also used for our personal
 * communication with the xavier.
 * 
 * Extend this class and implement messageReceiveCallback if you
 * want to use this serial protocol on a serial line.
 *
 * Structure of a Serial Message:
 * | Byte Number     | Byte Description                                           |
 * |:---------------:|------------------------------------------------------------|
 * | Frame Header    |                                                            |
 * | 0               | Frame Head Byte (0xA5)                                     |
 * | 1               | Frame Data Length, LSB                                     |
 * | 2               | Frame Data Length, MSB                                     |
 * | 3               | Frame Sequence Number                                      |
 * | 4               | CRC8 of the frame, (bytes 0 - 3)                           |
 * | 5               | Message Type, LSB                                          |
 * | 6               | Message Type, MSB                                          |
 * | Frame Body      | Data Length bytes                                          |
 * | 7 + Data Length | CRC16 of header and frame, LSB (bytes 0 - 6 + Data Length) |
 * | 8 + Data Length | CRC16 of header and frame, MSB                             |
 */
class DJISerial
{
 private:
    static const uint16_t SERIAL_RX_BUFF_SIZE = 256;
    static const uint16_t SERIAL_TX_BUFF_SIZE = 256;
    static const uint16_t SERIAL_HEAD_BYTE = 0xA5;
    static const uint8_t FRAME_DATA_LENGTH_OFFSET = 1;
    static const uint8_t FRAME_SEQUENCENUM_OFFSET = 3;
    static const uint8_t FRAME_CRC8_OFFSET = 4;
    static const uint8_t FRAME_HEADER_LENGTH = 7;
    static const uint8_t FRAME_TYPE_OFFSET = 5;
    static const uint8_t FRAME_CRC16_LENGTH = 2;

 public:
    /**
     * A container for storing TX and RX messages.
     */
    struct SerialMessage
    {
        uint8_t headByte;  ///< Use SERIAL_HEAD_BYTE.
        uint16_t length;  ///< Must be less than SERIAL_RX_BUFF_SIZE or SERIAL_TX_BUFF_SIZE.
        uint16_t type;  ///< The type is specified and interpreted by a derived class.
        uint8_t data[SERIAL_RX_BUFF_SIZE];
        modm::Timestamp messageTimestamp;  ///< The timestamp is in milliseconds.
        uint8_t sequenceNumber;  ///< A derived class may increment this for debugging purposes.
    };

    /**
     * Construct a Serial object.
     *
     * @param[in] port serial port to work on.
     * @param[in] isRxCRCEnforcementEnabled if to enable Rx CRC Enforcement.
     */
    DJISerial(
        Uart::UartPort port,
        bool isRxCRCEnforcementEnabled
    );

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
    void initialize();

    /**
     * Receive messages. Call periodically in order to receive all
     * incoming messages.
     *
     * @note tested with a delay of 10 microseconds with referee system. The
     *      longer the timeout the more likely a message failure may occur.
     */
    void updateSerial();

    /**
     * Called when a complete message is received. A derived class must
     * implement this in order to handle incoming messages properly.
     * 
     * @param[in] completeMessage a reference to the full message that has
     *      just been received by this class.
     */
    virtual void messageReceiveCallback(const SerialMessage& completeMessage) = 0;

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
    bool verifyCRC16(uint8_t *message, uint32_t messageLength, uint16_t expectedCRC16);

    /**
     * Calculate CRC8 of given array and compare against expectedCRC8.
     *
     * @param[in] data array to calculate CRC8.
     * @param[in] length length of array to check.
     * @param[in] expectedCRC8 expected CRC8.
     * @return if the calculated CRC8 matches CRC8 given.
     */
    bool verifyCRC8(uint8_t *message, uint32_t messageLength, uint8_t expectedCRC8);

    uint32_t read(uint8_t *data, uint16_t length);

    uint32_t write(const uint8_t *data, uint16_t length);

 protected:
    /**
     * Subclasses can access the message that this class sends as to allow
     * for modification.
     */
    SerialMessage txMessage;

    uint8_t txSequenceNumber;

    /**
     * Send a Message. This constructs a message from the `txMessage`,
     * which is protected and should be manipulated by a derived class.
     *
     * @return true if succeed, false if failed.
     */
    bool send();
};

}  // namespace serial

}  // namespace aruwlib

#endif
