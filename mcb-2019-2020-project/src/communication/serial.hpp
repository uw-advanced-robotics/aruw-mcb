#ifndef __serial_h_
#define __serial_h_

#define SERIAL_RX_BUFF_SIZE 256
#define SERIAL_TX_BUFF_SIZE 256

#define SERIAL_HEAD_BYTE 0xA5
#define SERIAL_FOOTER_LENGTH 2

#include "stdbool.h"

typedef void (*serial_message_handler_t)(uint16_t message_type, uint8_t* data_buffer, uint16_t length);

typedef enum {
	PORT_UART1 = 0,
	PORT_UART2 = 1,
	PORT_UART6 = 2,
} SERIAL_PORT;

typedef enum {
	WAITING_FOR_HEAD_BYTE = 0, 			// head byte (1-byte)
	WAITING_FOR_MESSAGE_LENGTH = 1,	// length of message (2-byte)
	WAITING_FOR_MESSAGE_DATA = 2,		// rest of data in packet [1-byte sequence num, 1-byte CRC8, message_length-byte message, 2-byte CRC16]
} SERIAL_MODE;

class Serial
{
public:
    Serial(SERIAL_PORT port, serial_message_handler_t message_handler);
    Serial();
    ~Serial();

    bool send(uint16_t message_type, uint16_t length, uint8_t* message_data);
    void update();
    void enableRXCRCEnforcement();

private:

    SERIAL_PORT port;

    // tx/rx buffers
	uint8_t buff_rx[SERIAL_RX_BUFF_SIZE];
	uint8_t buff_tx[SERIAL_TX_BUFF_SIZE];
	
	// state information
	SERIAL_MODE current_mode;

	// data read from an incoming message header
	uint16_t expected_message_length;
	uint16_t message_type;
	
	// handle electrical noise
	bool rxCRCEnforcementEnabled;
	uint8_t tx_sequence_num;
	uint8_t CRC8;
	uint16_t CRC16;
	uint8_t buff_CRC[SERIAL_RX_BUFF_SIZE];

    // message handler
	serial_message_handler_t handler;

    void serial_transition_to_mode(SERIAL_MODE new_mode);
    void process_receive();
    bool verifyCRC();

};







#endif