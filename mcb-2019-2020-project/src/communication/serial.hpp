#ifndef __serial_h_
#define __serial_h_

#define SERIAL_RX_BUFF_SIZE 256
#define SERIAL_TX_BUFF_SIZE 256

#define SERIAL_HEAD_BYTE 0xA5

#include <rm-dev-board-a/board.hpp>
#include "modm/processing.hpp"
/**
 * Structure of a Serial Message:
 * Frame Head{[Frame Head Byte(0xA5) 1 Byte][Frame Data Length (HSB Side Byte First, LSB Second) 2 Byte]
 *         [Frame Sequence Number 1 Byte][CRC8 of Bytes before 1 Byte]}
 * Frame Body{[Message Type (HSB Side Byte First, LSB Second) 2 Byte][Frame Data][CRC16 of entire frame (HSB Side Byte First, LSB Second) 2 Byte]}
 * 
 */
namespace aruwlib
{
namespace serial
{

class DJISerial
{
 public:
    typedef enum
    {
        // PORT_UART1 = 0,
        PORT_UART2 = 1,
        PORT_UART6 = 2,
    } Serial_Port;

    typedef enum
    {
        WAITING_FOR_HEAD_BYTE = 0,       // head byte (1-byte)
        WAITING_FOR_MESSAGE_LENGTH = 1,  // length of message (2-byte)
        WAITING_FOR_MESSAGE_DATA = 2,    // rest of data in packet [1-byte sequence num,
                                         // 1-byte CRC8, messageLength-byte message, 2-byte CRC16]
    } Serial_Mode;

    typedef struct{
        uint16_t type;
        uint16_t length;
        uint8_t* data;
    } Serial_Message_t;

    typedef void (*SerialMessageHandler_t)(Serial_Message_t* message);
    /**
     * Construct a Serial object
     * @param port serial port to work on
     * @param messageHandler callback function for handling received message
     * @param isRxCRCEnforcementEnabled if to enable Rx CRC Enforcement
     */
    DJISerial(
        Serial_Port port, SerialMessageHandler_t messageHandler,
        bool isRxCRCEnforcementEnabled
    );
    ~DJISerial();

    /**
     * Initialize serial
    */
    void initialize();

    /**
     * Send a Message
     * @param message pointer to message to send
     * @return true if succeed, false if failed
    */
    bool send(const Serial_Message_t* message);

    /**
     * Update the port, read a message from rx buffer and decode it
     * @param message pointer to output message
     * @return true if has new message, false if not
    */
    bool periodicTask(Serial_Message_t* message);

    /**
     * Enable RX CRC enforcement. Messages that don't pass CRC check will be ignored
    */
    void enableRxCRCEnforcement();

    /**
     * Enable RX CRC enforcement. Messages that don't pass CRC check will not be ignored
    */
    void disableRxCRCEnforcement();

    /**
     * Get current Timestamp
     * @return current Timestamp in ms
    */
    uint32_t getTimestamp();

    /**
     * Get timestamp of last message sent
     * @return timestamp of last message sent in ms
    */
    uint32_t getLastTxMessageTimestamp();

    /**
     * Get timestamp of last message received
     * @return timestamp of last message sent in ms
    */
    uint32_t getLastRxMessageTimestamp();

    /**
     * Get current Tx message sequence Number
     * @return current Tx message sequence Number
    */
    uint8_t getTxSequenceNumber();
 private:
    static const uint8_t FRAME_SOF_OFFSET = 0;
    static const uint8_t FRAME_DATA_LENGTH_OFFSET = 1;
    static const uint8_t FRAME_SEQUENCENUM_OFFSET = 3;
    static const uint8_t FRAME_CRC8_OFFSET = 4;
    static const uint8_t FRAME_HEADER_LENGTH = 5;
    static const uint8_t FRAME_TYPE_LENGTH = 2;
    static const uint8_t FRAME_TYPE_OFFSET = 5;
    static const uint8_t FRAME_DATA_OFFSET = 7;
    static const uint8_t FRAME_CRC16_LENGTH = 2;

    Serial_Port port;

    // tx/rx buffers
    uint8_t rxBuffer[SERIAL_RX_BUFF_SIZE];
    uint8_t txBuffer[SERIAL_TX_BUFF_SIZE];

    // state information
    Serial_Mode currentMode;

    // data read from an incoming message header
    uint16_t currentExpectedMessageLength;
    uint16_t currentMessageType;

    // handle electrical noise
    bool rxCRCEnforcementEnabled;
    uint8_t txSequenceNumber;
    uint8_t rxSequenceNumber;
    uint8_t CRC8;
    uint16_t CRC16;

    // message handler
    SerialMessageHandler_t handler;

    void switchToMode(Serial_Mode new_mode);
    bool processFrameHeader();
    bool processFrameData();

    uint32_t read(uint8_t *data, uint16_t length);
    uint32_t write(const uint8_t *data, uint16_t length);

    bool verifyCRC16(uint8_t *message, uint32_t messageLength, uint16_t expectedCRC16);
    bool verifyCRC8(uint8_t *message, uint32_t messageLength, uint8_t expectedCRC8);

    uint32_t lastTxMessageTimestamp;

    Serial_Message_t lastRxMessage;
    uint32_t lastRxMessageTimestamp;
};

}  // namespace serial

}  // namespace aruwlib

#endif
