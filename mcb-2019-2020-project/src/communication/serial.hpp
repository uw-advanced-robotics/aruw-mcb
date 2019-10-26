#ifndef __serial_h_
#define __serial_h_

#define SERIAL_RX_BUFF_SIZE 256
#define SERIAL_TX_BUFF_SIZE 256

#define SERIAL_HEAD_BYTE 0xA5
#define SERIAL_FOOTER_LENGTH 2

#include <rm-dev-board-a/board.hpp>
#include <stdbool.h>

typedef enum
{
    // PORT_UART1 = 0,
    PORT_UART2 = 1,
    PORT_UART6 = 2,
} SerialPort;

typedef enum
{
    WAITING_FOR_HEAD_BYTE = 0,      // head byte (1-byte)
    WAITING_FOR_MESSAGE_LENGTH = 1, // length of message (2-byte)
    WAITING_FOR_MESSAGE_DATA = 2,   // rest of data in packet [1-byte sequence num,
                                    // 1-byte CRC8, message_length-byte message, 2-byte CRC16]
} SerialMode;

typedef struct{
    uint16_t type;
    uint16_t length;
    uint8_t* data;
} Serial_Message_t;

typedef void (*message_handler_t)(Serial_Message_t* message);

class Serial
{
 public:
    Serial(SerialPort port, message_handler_t message_handler);
    Serial();
    ~Serial();

    void initialize();
    bool send(Serial_Message_t* message);
    void update();
    void enableRXCRCEnforcement();
    bool TXMessageRateReady(uint32_t previousTxMessageTimestamp, uint32_t minTxMessageInterval);
    uint32_t getTimestamp();
 private:
    SerialPort port;

    // tx/rx buffers
    uint8_t buff_rx[SERIAL_RX_BUFF_SIZE];
    uint8_t buff_tx[SERIAL_TX_BUFF_SIZE];

    // state information
    SerialMode current_mode;

    // data read from an incoming message header
    uint16_t expected_message_length;
    uint16_t message_type;

    // handle electrical noise
    bool rxCRCEnforcementEnabled;
    uint8_t tx_sequence_num;
    uint8_t rx_sequence_num;
    uint8_t CRC8;
    uint16_t CRC16;
    uint8_t buff_CRC[SERIAL_RX_BUFF_SIZE];

    // message handler
    message_handler_t handler;

    void switchToMode(SerialMode new_mode);
    void processReceive();
    bool verifyCRC();

    bool read(uint8_t *data, uint16_t length);
    bool write(const uint8_t *data, uint16_t length);
    uint16_t getRxBufferSize();

    uint32_t verifyCRC16(uint8_t *message, uint32_t message_length);
    uint32_t verifyCRC8(uint8_t *message, uint32_t message_length);

    modm::Timestamp timestamp;
};

#endif
