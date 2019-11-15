#include <stdint.h>
#include <string.h>
#include "serial.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"

namespace aruwlib
{
namespace serial
{

DJISerial::DJISerial(Serial_Port port, SerialMessageHandler_t messageHandler, bool isRxCRCEnforcementEnabled) {
    this->port = port;
    this->expected_message_length = 0;
    this->handler = messageHandler;
    this->message_type = 0;
    this->tx_sequence_num = 0;
    this->rx_sequence_num = 0;
    this->CRC8 = 0;
    this->CRC16 = 0;
    this->rxCRCEnforcementEnabled = isRxCRCEnforcementEnabled;
    this->timestamp = modm::Timestamp();
}

DJISerial::~DJISerial() {
}


/**
 * Calculate CRC8 of given array and compare against expectedCRC8
 * @param data array to calculate CRC8
 * @param length length of array to check
 * @param expectedCRC8 expected CRC8
 * @return if the calculated CRC8 matches CRC8 given
 */
bool DJISerial::verifyCRC8(uint8_t *data, uint32_t length, uint8_t expectedCRC8) {
    uint8_t tempCRC8 = 0;
    if ((data == NULL) || (length <= 0))
    {
        return 0;
    }
    tempCRC8 = aruwlib::math::calculateCRC8(data, length, CRC8_INIT);
    return tempCRC8 == expectedCRC8;
}

/**
 * Calculate CRC16 of given array and compare against expectedCRC16
 * @param data array to calculate CRC16
 * @param length length of array to check
 * @param expectedCRC16 expected CRC16
 * @return if the calculated CRC16 matches CRC16 given
 */
bool DJISerial::verifyCRC16(uint8_t *data, uint32_t length, uint16_t expectedCRC16) {
    uint16_t tempCRC16 = 0;
    if ((data == NULL) || (length <= 0))
    {
        return false;
    }
    tempCRC16 = aruwlib::math::calculateCRC16(data, length, CRC16_INIT);
    return tempCRC16 == expectedCRC16;
}


bool DJISerial::processFrameHeader() {
    buff_rx[1] = 0;
    buff_rx[2] = 0;
    buff_rx[3] = 0;
    buff_rx[4] = 0;
    this->read(buff_rx + 1, FRAME_HEADER_LENGTH - 1);// Skip the head byte
    expected_message_length = (buff_rx[FRAME_DATA_LENGTH_OFFSET + 1] << 8) | buff_rx[FRAME_DATA_LENGTH_OFFSET];
    rx_sequence_num = buff_rx[FRAME_SEQUENCENUM_OFFSET];
    CRC8 = buff_rx[FRAME_CRC8_OFFSET];
    // the message length must be greater than 0 and less than the max
    // size of the buffer - 9 (9 bytes are used for the packet header and CRC)
    if (expected_message_length > 0){// && expected_message_length
        //<= SERIAL_RX_BUFF_SIZE - (FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH)) {
        if(!rxCRCEnforcementEnabled) {
            return processFrameData();
        }
        else{
            if(verifyCRC8(buff_rx, FRAME_HEADER_LENGTH - 1, CRC8)){
                return processFrameData();
            }
            else{ 
                expected_message_length = 0;
                return false;
            }
            
        }
    } else {
        expected_message_length = 0;
        return false;
        // if the length isn't valid, throw out the message and go back
        // to scanning for messages
    }
}


bool DJISerial::processFrameData() {
    // get the rest of the packet (1-byte sequence, 1-byte CRC8, 2-byte
    // message type, message data, 2-byte CRC16)
    // skip frame header
    this->read((buff_rx + FRAME_HEADER_LENGTH), expected_message_length +
            FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH);

    message_type = (buff_rx[FRAME_TYPE_OFFSET + 1] << 8) | buff_rx[FRAME_TYPE_OFFSET];
    
    CRC16 = ((buff_rx[FRAME_DATA_OFFSET + expected_message_length + 1] << 8)
        | buff_rx[FRAME_DATA_OFFSET + expected_message_length]);

    // if CRC checking is not enabled, process the message, otherwise
    // check if CRC8/CRC16 are valid
    if (!rxCRCEnforcementEnabled) {
        // CRC checking is disabled for this huart port
        Serial_Message_t message;
        message.length = expected_message_length;
        message.data = buff_rx + FRAME_DATA_OFFSET;
        message.type = message_type;
        this->lastMessage = message;
        handler(&message);
        return true;
    } else if (verifyCRC16(buff_rx, FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH +
                        expected_message_length, CRC16)) {
        // CRC checking is enabled and CRC8/CRC16 were valid
        Serial_Message_t message;
        message.length = expected_message_length;
        message.data = buff_rx + FRAME_DATA_OFFSET;
        message.type = message_type;
        this->lastMessage = message;
        handler(&message);

        return true;
    } else {
        // CRC checking is enabled but CRC8/CRC16 were invalid
        expected_message_length = 0;
        return false;
    }
}

bool DJISerial::send(Serial_Message_t* message) {
    uint8_t buff[SERIAL_TX_BUFF_SIZE];
    buff[0] = SERIAL_HEAD_BYTE;
    buff[1] = message->length & 0xFF;
    buff[2] = message->length >> 8;
    buff[3] = tx_sequence_num;
    buff[4] = aruwlib::math::calculateCRC8(buff, 4, CRC8_INIT);
    buff[5] = message->type & 0xFF;
    buff[6] = message->type >> 8;

    uint8_t *next_tx_buff = &(buff[7]);

    if (next_tx_buff + message->length + SERIAL_FOOTER_LENGTH >=buff + SERIAL_TX_BUFF_SIZE) {
        return false;
    }
    for (uint16_t i = 0; i < message->length; i++) {
        *next_tx_buff = message->data[i];
        next_tx_buff++;
    }
    uint16_t CRC16_val = aruwlib::math::calculateCRC16(buff, FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH + message->length, CRC16_INIT);
    next_tx_buff[0] = CRC16_val & 0xFF;
    next_tx_buff[1] = CRC16_val >> 8;
    next_tx_buff += SERIAL_FOOTER_LENGTH;
    uint16_t total_size = next_tx_buff - buff;
    bool status = this->write(buff, total_size);
    if (status) {
        return false;
    }
    tx_sequence_num++;
    return true;
}

void DJISerial::enableRxCRCEnforcement() {
    rxCRCEnforcementEnabled = true;
}
void DJISerial::disableRxCRCEnforcement() {
    rxCRCEnforcementEnabled = false;
}

bool DJISerial::update(Serial_Message_t* message) {
    uint8_t data;
    while (read(&data, 1))
    {
        if (data == SERIAL_HEAD_BYTE)
        {
            buff_rx[0] = SERIAL_HEAD_BYTE;
            if(processFrameHeader()) {
                message->type = lastMessage.type;
                message->length = lastMessage.length;
                message->data = lastMessage.data;
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
    //temporary solution
}

uint32_t DJISerial::getTimestamp() {
    return timestamp.getTime();
}

bool DJISerial::TxMessageRateReady(
    uint32_t previousTxMessageTimestamp,
    uint32_t minTxMessageInterval
) {
    uint32_t currentTime = timestamp.getTime();
    if (
        previousTxMessageTimestamp == 0
        || currentTime - previousTxMessageTimestamp > minTxMessageInterval
    ) {
        previousTxMessageTimestamp = currentTime;
        return true;
    } else {
        return false;
    }
}

bool DJISerial::read(uint8_t *data, uint16_t length) {
    switch (this->port) {
    case PORT_UART2:
        return Usart2::read(data, length);
    case PORT_UART6:
        return Usart6::read(data, length);
    default:
        return false;
    }
}
bool DJISerial::write(const uint8_t *data, uint16_t length) {
    switch (this->port) {
    case PORT_UART2:
        return Usart2::write(data, length);
    case PORT_UART6:
        return Usart6::write(data, length);
    default:
        return false;
    }
}
void DJISerial::initialize() {
    switch (this->port) {
    case PORT_UART2:
        Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
        Usart2::initialize<Board::SystemClock, 115200>();
    case PORT_UART6:
        Usart6::connect<GpioG14::Tx, GpioG9::Rx>();
        Usart6::initialize<Board::SystemClock, 115200>();
    default:
        break;
    }
}

uint8_t DJISerial::getTxSequenceNumber(){
    return this->tx_sequence_num;
}

}
}