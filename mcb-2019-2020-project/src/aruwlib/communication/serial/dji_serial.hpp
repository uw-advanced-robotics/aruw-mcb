#ifndef __serial_h_
#define __serial_h_

#include <modm/processing.hpp>
#include <rm-dev-board-a/board.hpp>

/**
 * Structure of a Serial Message:
 * Frame Head{
 * [Frame Head Byte(0xA5) 1 Byte]
 * [Frame Data Length (HSB Side Byte First, LSB Second) 2 Byte]
 * [Frame Sequence Number 1 Byte]
 * [CRC8 of Bytes before 1 Byte]
 * }
 * Frame Body{
 * [Message Type (HSB Side Byte First, LSB Second) 2 Byte]
 * [Frame Data]
 * [CRC16 of entire frame (HSB Side Byte First, LSB Second) 2 Byte]
 * }
 * 
 */
namespace aruwlib
{

namespace serial
{

class DJISerial
{
 private:
    static const int16_t SERIAL_RX_BUFF_SIZE = 256;
    static const int16_t SERIAL_TX_BUFF_SIZE = 256;
    static const int16_t SERIAL_HEAD_BYTE = 0xA5;
    static const uint8_t FRAME_SOF_OFFSET = 0;
    static const uint8_t FRAME_DATA_LENGTH_OFFSET = 1;
    static const uint8_t FRAME_SEQUENCENUM_OFFSET = 3;
    static const uint8_t FRAME_CRC8_OFFSET = 4;
    static const uint8_t FRAME_HEADER_LENGTH = 5;
    static const uint8_t FRAME_TYPE_LENGTH = 2;
    static const uint8_t FRAME_TYPE_OFFSET = 5;
    static const uint8_t FRAME_DATA_OFFSET = 7;
    static const uint8_t FRAME_CRC16_LENGTH = 2;

 public:
    typedef enum
    {
        // PORT_UART1 = 0,
        PORT_UART2 = 1,
        PORT_UART6 = 2,
    } SerialPort;

    enum SerialRxState
    {
        SERIAL_HEADER_SEARCH,
        PROCESS_FRAME_HEADER,
        PROCESS_FRAME_MESSAGE_TYPE,
        PROCESS_FRAME_DATA
    };

    typedef struct
    {
        uint16_t length;
        uint8_t headByte;
        uint16_t type;
        uint8_t data[SERIAL_RX_BUFF_SIZE];
        modm::Timestamp messageTimestamp;
        uint8_t sequenceNumber;
    } SerialMessage_t;

    typedef void (*SerialMessageHandler)(SerialMessage_t * const message);

    /**
     * Construct a Serial object
     * @param port serial port to work on
     * @param messageHandler callback function for handling received message
     * @param isRxCRCEnforcementEnabled if to enable Rx CRC Enforcement
     */
    DJISerial(
        SerialPort port,
        bool isRxCRCEnforcementEnabled
    );

    /**
     * Initialize serial
     */
    void initialize();

    /**
     * Send a Message
     * @param message pointer to message to send
     * @return true if succeed, false if failed
     */
    bool send(void);

    /**
     * Receive messages. Call periodically in order to not miss any messages
     */
    void updateSerial(void);

 private:
    // serial port you are connected to
    SerialPort port;

    // stuff for rx, buffers to store parts of the header, state machine
    SerialRxState djiSerialRxState;
    SerialMessage_t newMessage;
    uint16_t frameCurrReadByte;
    uint8_t frameType[FRAME_TYPE_LENGTH];
    uint8_t frameHeader[FRAME_HEADER_LENGTH];
    // handle electrical noise
    bool rxCRCEnforcementEnabled;

    // tx buffer
    uint8_t txBuffer[SERIAL_TX_BUFF_SIZE];

    bool verifyCRC16(uint8_t *message, uint32_t messageLength, uint16_t expectedCRC16);

    bool verifyCRC8(uint8_t *message, uint32_t messageLength, uint8_t expectedCRC8);

    uint32_t read(uint8_t *data, uint16_t length);

    uint32_t write(const uint8_t *data, uint16_t length);

 protected:  // subclasses can access the most recent message and the message that this
             // class sends
    SerialMessage_t txMessage;
    SerialMessage_t mostRecentMessage;
};

}  // namespace serial

}  // namespace aruwlib

#endif
