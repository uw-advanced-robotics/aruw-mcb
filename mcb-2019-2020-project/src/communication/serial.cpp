#include <stdint.h>
#include <string.h>
#include "serial.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"

namespace aruwlib
{
namespace serial
{

DJISerial::DJISerial(
    Serial_Port port, SerialMessageHandler_t messageHandler,
    bool isRxCRCEnforcementEnabled
) {
    this->port = port;
    this->currentExpectedMessageLength = 0;
    this->handler = messageHandler;
    this->currentMessageType = 0;
    this->txSequenceNumber = 0;
    this->rxSequenceNumber = 0;
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
    if (data == NULL)
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
    if (data == NULL)
    {
        return false;
    }
    tempCRC16 = aruwlib::math::calculateCRC16(data, length, CRC16_INIT);
    return tempCRC16 == expectedCRC16;
}


bool DJISerial::processFrameHeader() {
    rxBuffer[1] = 0;
    rxBuffer[2] = 0;
    rxBuffer[3] = 0;
    rxBuffer[4] = 0;
    // Skip the head byte
    uint32_t actualLength = this->read(rxBuffer + 1, FRAME_HEADER_LENGTH - 1);
    if (actualLength != FRAME_HEADER_LENGTH - 1)
    {
        return false;
    }

    currentExpectedMessageLength = (rxBuffer[FRAME_DATA_LENGTH_OFFSET + 1] << 8)
        | rxBuffer[FRAME_DATA_LENGTH_OFFSET];
    rxSequenceNumber = rxBuffer[FRAME_SEQUENCENUM_OFFSET];
    CRC8 = rxBuffer[FRAME_CRC8_OFFSET];
    // the message length must be greater than 0 and less than the max
    // size of the buffer - 9 (9 bytes are used for the packet header and CRC)
    if (currentExpectedMessageLength > 0) {  // && expected_message_length
        //<= SERIAL_RX_BUFF_SIZE - (FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH)) {
        if(!rxCRCEnforcementEnabled) {
            return processFrameData();
        } else{
            if(verifyCRC8(rxBuffer, FRAME_HEADER_LENGTH - 1, CRC8)) {
                return processFrameData();
            } else{
                currentExpectedMessageLength = 0;
                return false;
            }
        }
    } else {
        currentExpectedMessageLength = 0;
        return false;
        // if the length isn't valid, throw out the message and go back
        // to scanning for messages
    }
}


bool DJISerial::processFrameData() {
    // get the rest of the packet (1-byte sequence, 1-byte CRC8, 2-byte
    // message type, message data, 2-byte CRC16)
    // skip frame header
    uint32_t actualLength = this->read((rxBuffer + FRAME_HEADER_LENGTH),
        currentExpectedMessageLength
        + FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH);
    if (actualLength != (uint32_t)(currentExpectedMessageLength
        + FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH)
    ) {
        return false;
    }
    currentMessageType = (rxBuffer[FRAME_TYPE_OFFSET + 1] << 8) | rxBuffer[FRAME_TYPE_OFFSET];

    CRC16 = ((rxBuffer[FRAME_DATA_OFFSET + currentExpectedMessageLength + 1] << 8)
        | rxBuffer[FRAME_DATA_OFFSET + currentExpectedMessageLength]);

    // if CRC checking is not enabled, process the message, otherwise
    // check if CRC8/CRC16 are valid
    if (!rxCRCEnforcementEnabled) {
        // CRC checking is disabled for this huart port
        Serial_Message_t message;
        message.length = currentExpectedMessageLength;
        message.data = rxBuffer + FRAME_DATA_OFFSET;
        message.type = currentMessageType;
        this->lastRxMessage = message;
        lastRxMessageTimestamp = timestamp.getTime();
        handler(&message);
        return true;
    } 
    if (verifyCRC16(rxBuffer, FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH +
                        currentExpectedMessageLength, CRC16)) {
        // CRC checking is enabled and CRC8/CRC16 were valid
        Serial_Message_t message;
        message.length = currentExpectedMessageLength;
        message.data = rxBuffer + FRAME_DATA_OFFSET;
        message.type = currentMessageType;
        this->lastRxMessage = message;
        lastRxMessageTimestamp = timestamp.getTime();
        handler(&message);

        return true;
    // CRC checking is enabled but CRC8/CRC16 were invalid
    currentExpectedMessageLength = 0;
    return false;
    }
}

bool DJISerial::send(const Serial_Message_t* message) {
    txBuffer[0] = SERIAL_HEAD_BYTE;
    txBuffer[1] = message->length;
    txBuffer[2] = message->length >> 8;
    txBuffer[3] = txSequenceNumber;
    txBuffer[4] = aruwlib::math::calculateCRC8(txBuffer, 4, CRC8_INIT);
    txBuffer[5] = message->type;
    txBuffer[6] = message->type >> 8;

    uint8_t *next_tx_buff = &(txBuffer[7]);

    if (next_tx_buff + message->length + FRAME_CRC16_LENGTH >= txBuffer + SERIAL_TX_BUFF_SIZE) {
        return false;
    }
    for (uint16_t i = 0; i < message->length; i++) {
        *next_tx_buff = message->data[i];
        next_tx_buff++;
    }
    uint16_t CRC16_val = aruwlib::math::calculateCRC16(
        txBuffer,
        FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH + message->length, CRC16_INIT
    );
    next_tx_buff[0] = CRC16_val;
    next_tx_buff[1] = CRC16_val >> 8;
    next_tx_buff += FRAME_CRC16_LENGTH;
    uint16_t total_size = next_tx_buff - txBuffer;
    uint32_t actual_length = this->write(txBuffer, total_size);
    if (actual_length != total_size) {
        return false;
    }
    txSequenceNumber++;
    lastTxMessageTimestamp = timestamp.getTime();
    return true;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void DJISerial::enableRxCRCEnforcement() {
    rxCRCEnforcementEnabled = true;
}
void DJISerial::disableRxCRCEnforcement() {
    rxCRCEnforcementEnabled = false;
}

bool DJISerial::periodicTask(Serial_Message_t* message) {
    uint8_t data;
    while (read(&data, 1))
    {
        if (data == SERIAL_HEAD_BYTE)
        {
            rxBuffer[0] = SERIAL_HEAD_BYTE;
            if(processFrameHeader()) {
                message->type = lastRxMessage.type;
                message->length = lastRxMessage.length;
                message->data = lastRxMessage.data;
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}

uint32_t DJISerial::getTimestamp() {
    return timestamp.getTime();
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

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
uint8_t DJISerial::getTxSequenceNumber() {
    return this->txSequenceNumber;
}

uint32_t DJISerial::getLastTxMessageTimestamp() {
    return this->lastTxMessageTimestamp;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
uint32_t DJISerial::getLastRxMessageTimestamp() {
    return this->lastRxMessageTimestamp;
}

}  // namespace serial

}  // namespace aruwlib
