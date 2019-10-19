
#include <stdint.h>
#include <string.h>
#include "serial.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"



uint32_t verifyCRC8(uint8_t *message, uint32_t message_length) {
    uint8_t CRC8_expected = 0;
    if ((message == NULL) || (message_length <= 0)) {return 0;}
    CRC8_expected = calculateCRC8(message, message_length, CRC8_INIT);
    return ( CRC8_expected == message[message_length] );
}

uint32_t verifyCRC16(uint8_t *message, uint32_t message_length) {
    uint16_t CRC16_expected = 0;
    if ((message == NULL) || (message_length <= 0)) {
        return false;
    }
    CRC16_expected = calculateCRC16(message, message_length, CRC16_INIT);
    return ((CRC16_expected & 0xff) == message[message_length] && 
    ((CRC16_expected >> 8) & 0xff) == message[message_length + 1]);
}

bool Serial::verifyCRC() {
	return verifyCRC8(buff_CRC, 4) && 
		verifyCRC16(buff_CRC, 7 + expected_message_length);
}

void Serial::process_receive() {
	switch(current_mode) {
		case WAITING_FOR_HEAD_BYTE:
			if (buff_rx[0] != SERIAL_HEAD_BYTE) {
				// if this byte isn't the header byte, just keep scanning
				serial_transition_to_mode(WAITING_FOR_HEAD_BYTE);
				break;
			}
			serial_transition_to_mode(WAITING_FOR_MESSAGE_LENGTH);
			break;
			
		case WAITING_FOR_MESSAGE_LENGTH:
			expected_message_length = (buff_rx[1] << 8) | buff_rx[0];
			// the message length must be greater than 0 and less than the max size of the buffer	 - 9 (9 bytes are used for the packet header and CRC)
			if (expected_message_length > 0 && expected_message_length <= SERIAL_RX_BUFF_SIZE - 9) {
				serial_transition_to_mode(WAITING_FOR_MESSAGE_DATA);
			} else {
				expected_message_length = 0;
				// if the length isn't valid, throw out the message and go back to scanning for messages
				serial_transition_to_mode(WAITING_FOR_HEAD_BYTE);
			}
			break;
			
		case WAITING_FOR_MESSAGE_DATA:
			rx_sequence_num =  buff_rx[0];
			CRC8 = buff_rx[1];
			message_type = (buff_rx[3] << 8) | buff_rx[2];
			CRC16 = (buff_rx[5 + expected_message_length] << 8) | buff_rx[4 + expected_message_length];
			
			// set CRC bufffer to get CRC8 and CRC16 values
			buff_CRC[0] = SERIAL_HEAD_BYTE;
			*((uint16_t*) (buff_CRC + 1)) = expected_message_length;
			memcpy(buff_CRC + 3, buff_rx, 6 + expected_message_length);
		  
			// if CRC checking is not enabled, process the message, otherwise check if CRC8/CRC16 are valid
			if (!rxCRCEnforcementEnabled) {
				// CRC checking is disabled for this huart port
				handler(message_type, (buff_rx + 4), expected_message_length);
			} else if (verifyCRC()) {
				// CRC checking is enabled and CRC8/CRC16 were valid
				handler(message_type, (buff_rx + 4), expected_message_length);
			} else {
				// CRC checking is enabled but CRC8/CRC16 were invalid
				expected_message_length = 0;
			}
			
			serial_transition_to_mode(WAITING_FOR_HEAD_BYTE);
			break;
			
		default:
			break;
	}
}

void Serial::serial_transition_to_mode(SERIAL_MODE new_mode) {
	switch (new_mode) {
		case WAITING_FOR_HEAD_BYTE:
			Board::Led1::setOutput(modm::Gpio::High);
			Board::Led3::setOutput(modm::Gpio::Low);
			Board::Led6::setOutput(modm::Gpio::Low);
			current_mode = WAITING_FOR_HEAD_BYTE;
			// search through first byte for head byte
			if (Usart6::getRxBufferSize() >= 1) {
				Usart6::read(buff_rx, 1);
			}
			else { //data not yet available
				current_mode = WAITING_FOR_HEAD_BYTE; //go back to previous mode
			}
			
			break;

		case WAITING_FOR_MESSAGE_LENGTH:
			Board::Led1::setOutput(modm::Gpio::Low);
			Board::Led3::setOutput(modm::Gpio::High);
			Board::Led6::setOutput(modm::Gpio::Low);
			current_mode = WAITING_FOR_MESSAGE_LENGTH;
			// zero out the first 16 bits of the rx buffer so we can validate we got data
			buff_rx[0] = 0;
			buff_rx[1] = 0;
			// get next 2 bytes for the message length
			if (Usart6::getRxBufferSize() >= 2) {
				Usart6::read(buff_rx, 2);
			}
			else { //data not yet available
				current_mode = WAITING_FOR_MESSAGE_LENGTH; //go back to previous mode
			}
			
			break;

		case WAITING_FOR_MESSAGE_DATA:
			Board::Led1::setOutput(modm::Gpio::Low);
			Board::Led3::setOutput(modm::Gpio::Low);
			Board::Led6::setOutput(modm::Gpio::High);
			current_mode = WAITING_FOR_MESSAGE_DATA;
			// get the rest of the packet (1-byte sequence, 1-byte CRC8, 2-byte message type, message data, 2-byte CRC16)
			if (Usart6::getRxBufferSize() >= expected_message_length + 6) {
				Usart6::read(buff_rx, expected_message_length + 6);
			}
			else { //data not yet available
				current_mode = WAITING_FOR_MESSAGE_DATA; //go back to previous mode
			}
			break;
		
		default:
			break;
	}
}

bool Serial::send(uint16_t message_type,uint16_t length,uint8_t* message_data){
    uint8_t buff[SERIAL_TX_BUFF_SIZE];
    buff[0] = SERIAL_HEAD_BYTE;
    buff[1] = length & 0xFF;
    buff[2] = length >> 8;
    buff[3] = tx_sequence_num;
    buff[4] = calculateCRC8(buff, 4, CRC8_INIT);
    buff[5] = message_type & 0xFF;
    buff[6] = message_type >> 8;

    uint8_t* next_tx_buff = &(buff[7]);

    if (next_tx_buff + length + SERIAL_FOOTER_LENGTH >= buff + SERIAL_TX_BUFF_SIZE) {
        return false;
    }
    for (uint16_t i = 0; i < length; i++) {
        *next_tx_buff = message_data[i];
        next_tx_buff++;
    }
    uint16_t CRC16_val = calculateCRC16(buff, 7 + length, CRC16_INIT);
    next_tx_buff[0] = CRC16_val & 0xFF;
    next_tx_buff[1] = CRC16_val >> 8;
    next_tx_buff += SERIAL_FOOTER_LENGTH;
    uint16_t total_size = next_tx_buff - buff;
    bool status = Usart6::write(buff, total_size);
    if (status) {
        return false;
    }
    tx_sequence_num++;
    return true;
}

Serial::Serial(SERIAL_PORT port, serial_message_handler_t message_handler) {

    Usart6::connect<GpioG14::Tx,GpioG9::Rx>();
    Usart6::initialize<Board::SystemClock,115200>();
    this->port = port;
	this->expected_message_length = 0;
	this->handler = message_handler;
	this->message_type = 0;
	this->tx_sequence_num = 0;
	this->rx_sequence_num = 0;
	this->CRC8 = 0;
	this->CRC16 = 0;
	this->rxCRCEnforcementEnabled = false;
	serial_transition_to_mode(WAITING_FOR_HEAD_BYTE);

}

Serial::Serial() {

	this->expected_message_length = 0;
	this->message_type = 0;
	this->tx_sequence_num = 0;
	this->CRC8 = 0;
	this->CRC16 = 0;
	this->rxCRCEnforcementEnabled = false;

}

Serial::~Serial(){
    
}


void Serial::enableRXCRCEnforcement() {
	rxCRCEnforcementEnabled = true;
}

void Serial::update() {
	if (Usart6::getRxBufferSize() > 0) {
		process_receive();
	}
}

bool Serial::TXMessageRateReady(uint32_t previousTxMessageTimestamp,  uint32_t minTxMessageInterval) {
	//uint32_t currentTime = 0; //modm::Timestamp.getTime(); //osKernelSysTick();
	//if (previousTxMessageTimestamp == 0
	//	  || currentTime - previousTxMessageTimestamp > minTxMessageInterval)
	//{
	//	previousTxMessageTimestamp = currentTime;
		return true;
	//} else {
	//	return false;
	//}
}