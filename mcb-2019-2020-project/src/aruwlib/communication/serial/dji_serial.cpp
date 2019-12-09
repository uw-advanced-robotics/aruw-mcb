#include <rm-dev-board-a/board.hpp>
#include "dji_serial.hpp"
#include "src/aruwlib/algorithms/crc.hpp"

namespace aruwlib
{
namespace serial
{

DJISerial::DJISerial(
    SerialPort port,
    SerialMessageHandler messageHandler,
    bool isRxCRCEnforcementEnabled
):
port(port),
currentExpectedMessageLength(0),
currentMessageType(0),
rxCRCEnforcementEnabled(isRxCRCEnforcementEnabled),
txSequenceNumber(0),
lastTxMessageTimestamp(0)
{}

void DJISerial::initialize() {
    switch (this->port) {
    case PORT_UART2:
        Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
        Usart2::initialize<Board::SystemClock, 115200>();
        break;
    case PORT_UART6:
        Usart6::connect<GpioG14::Tx, GpioG9::Rx>();
        Usart6::initialize<Board::SystemClock, 115200>();
        break;
    default:
        break;
    }
}

bool DJISerial::send(const SerialMessage* message) {
    txBuffer[0] = SERIAL_HEAD_BYTE;
    txBuffer[1] = message->length;
    txBuffer[2] = message->length >> 8;
    txBuffer[3] = txSequenceNumber;
    txBuffer[4] = algorithms::calculateCRC8(txBuffer, 4, CRC8_INIT);
    txBuffer[5] = message->type;
    txBuffer[6] = message->type >> 8;

    uint8_t *nextTxBuff = &(txBuffer[7]);

    // we can't send, trying to send too much
    if (nextTxBuff + message->length + FRAME_CRC16_LENGTH >= txBuffer + SERIAL_TX_BUFF_SIZE) {
        // NON-FATAL-ERROR-CHECK
        return false;
    }

    for (uint16_t i = 0; i < message->length; i++) {
        *nextTxBuff = message->data[i];
        nextTxBuff++;
    }

    uint16_t CRC16Val = algorithms::calculateCRC16(
        txBuffer,
        FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH + message->length, CRC16_INIT
    );

    nextTxBuff[0] = CRC16Val;
    nextTxBuff[1] = CRC16Val >> 8;
    nextTxBuff += FRAME_CRC16_LENGTH;
    uint32_t totalSize = nextTxBuff - txBuffer;
    uint32_t actualLength = this->write(txBuffer, totalSize);
    if (actualLength != totalSize) {
        return false;
    }
    txSequenceNumber++;
    lastTxMessageTimestamp = modm::Clock::now().getTime();
    return true;
}

void DJISerial::updateSerial() {
    switch(djiSerialState)
    {
        case SERIAL_HEADER_SEARCH:
        {
            // keep scanning for the head byte as long as you are here and have not yet found it.
            uint8_t serialHeadCheck = 0;
            while (read(&serialHeadCheck, 1) && djiSerialState == SERIAL_HEADER_SEARCH)
            {
                // we found it, store the head byte
                if (serialHeadCheck == SERIAL_HEAD_BYTE)
                {
                    frameHeader[0] = SERIAL_HEAD_BYTE;
                    djiSerialState = PROCESS_FRAME_HEADER;
                }
            }
            
            // recursively call youself if you have received the frame header. Important that we
            // don't miss any bytes coming in when polling.
            if (djiSerialState == PROCESS_FRAME_HEADER)
            {
                updateSerial();
            }
            break;
        }
        case PROCESS_FRAME_HEADER:  // the frame header consists of the length, type, and CRC8
        {
            // Read from the buffer. Keep track of the index in the frameHeader array using the 
            // frameCurrReadByte. +1 at beginning and -1 on the end since the serial head
            // byte is part of the frame but has already been processed.
            frameCurrReadByte += read(frameHeader + frameCurrReadByte + 1,
                FRAME_HEADER_LENGTH - frameCurrReadByte - 1);

            // We have the complete message header in the frameHeader buffer
            if (frameCurrReadByte == FRAME_HEADER_LENGTH)
            {
                frameCurrReadByte = 0;

                // process length
                newMessage.length = (frameHeader[FRAME_DATA_LENGTH_OFFSET + 1] << 8)
                    | frameHeader[FRAME_DATA_LENGTH_OFFSET];
                // process sequence number (counter)
                newMessage.sequenceNumber = frameHeader[FRAME_SEQUENCENUM_OFFSET];

                if (
                    newMessage.length == 0 || newMessage.length >= SERIAL_RX_BUFF_SIZE
                    - (FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH)
                ) {
                    djiSerialState = SERIAL_HEADER_SEARCH;
                    // THROW-NON-FATAL-ERROR-CHECK
                    return;
                }
                
                // check crc8 on header
                if (rxCRCEnforcementEnabled)
                {
                    uint8_t CRC8 = frameHeader[FRAME_CRC8_OFFSET];
                    if (!verifyCRC8(frameHeader, FRAME_HEADER_LENGTH - 1, CRC8))
                    {
                        djiSerialState = SERIAL_HEADER_SEARCH;
                        // THROW-NON-FATAL-ERROR-CHECK
                    }
                }

                // move on to processing message body
                djiSerialState = PROCESS_FRAME_DATA;
                
                // recursively call yourself so you don't miss any data that is in the process of
                // being received.
                updateSerial();
            }
            break;
        }
        case PROCESS_FRAME_MESSAGE_TYPE:  // here we read the next two bytes of the message
        {
            frameCurrReadByte = read(frameType + frameCurrReadByte ,
                FRAME_TYPE_LENGTH - frameCurrReadByte);
            if (frameCurrReadByte == FRAME_TYPE_LENGTH)
            {
                newMessage.type = (frameType[1] << 8) | frameType[0];

                djiSerialState = PROCESS_FRAME_DATA;

                updateSerial();
            }
            else if (frameCurrReadByte > 2)
            {
                frameCurrReadByte = 0;
                djiSerialState = SERIAL_HEADER_SEARCH;
                // THROW-NON-FATAL-ERROR-CHECK
            }
            break;
        }
        case PROCESS_FRAME_DATA:  // read bulk of message
        {
            // add on extra 2 bytes for crc enforcement, and read bytes until
            // the length has been reached
            if (rxCRCEnforcementEnabled)
            {
                frameCurrReadByte += read(newMessage.data + frameCurrReadByte,
                    newMessage.length + 2 - frameCurrReadByte);
            }
            else
            {
                frameCurrReadByte += read(newMessage.data + frameCurrReadByte,
                    newMessage.length - frameCurrReadByte);
            }

            if (frameCurrReadByte == newMessage.length)
            {
                frameCurrReadByte = 0;
                if (rxCRCEnforcementEnabled)
                {
                    uint16_t CRC16 = (newMessage.data[newMessage.length + 1] << 8)
                        | newMessage.data[newMessage.length];
                    if (false)  // todo(matthew) fix this
                    {
                        return;
                        // NON-FATAL-ERROR-CHECK
                    }
                }

                // update the time and copy over the message to the most recent message
                newMessage.messageTimestamp = modm::Clock::now();
                memcpy(&newMessage, &mostRecentMessage, sizeof(SerialMessage_t));

                djiSerialState = SERIAL_HEADER_SEARCH;
            }
            else if (frameCurrReadByte > newMessage.length)
            {
                frameCurrReadByte = 0;
                // THROW-NON-FATAL-ERROR-CHECK
                djiSerialState = SERIAL_HEADER_SEARCH;
            }
            break;
        }
    }
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
uint8_t DJISerial::getTxSequenceNumber() {
    return this->txSequenceNumber;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
uint32_t DJISerial::getLastTxMessageTimestamp() {
    return lastTxMessageTimestamp;
}


/**
 * Calculate CRC8 of given array and compare against expectedCRC8
 * @param data array to calculate CRC8
 * @param length length of array to check
 * @param expectedCRC8 expected CRC8
 * @return if the calculated CRC8 matches CRC8 given
 */
bool DJISerial::verifyCRC8(uint8_t *data, uint32_t length, uint8_t expectedCRC8) {
    uint8_t actualCRC8 = 0;
    if (data == NULL)
    {
        return false;
    }
    actualCRC8 = algorithms::calculateCRC8(data, length, CRC8_INIT);
    return actualCRC8 == expectedCRC8;
}

/**
 * Calculate CRC16 of given array and compare against expectedCRC16
 * @param data array to calculate CRC16
 * @param length length of array to check
 * @param expectedCRC16 expected CRC16
 * @return if the calculated CRC16 matches CRC16 given
 */
bool DJISerial::verifyCRC16(uint8_t *data, uint32_t length, uint16_t expectedCRC16) {
    uint16_t actualCRC16 = 0;
    if (data == NULL)
    {
        return false;
    }
    actualCRC16 = algorithms::calculateCRC16(data, length, CRC16_INIT);
    return actualCRC16 == expectedCRC16;
}

uint32_t DJISerial::read(uint8_t *data, uint16_t length) {
    switch (this->port) {
    case PORT_UART2:
        return Usart2::read(data, length);
    case PORT_UART6:
        return Usart6::read(data, length);
    default:
        return 0;
    }
}

uint32_t DJISerial::write(const uint8_t *data, uint16_t length) {
    switch (this->port) {
    case PORT_UART2:
        return Usart2::write(data, length);
    case PORT_UART6:
        return Usart6::write(data, length);
    default:
        return 0;
    }
}

}  // namespace serial

}  // namespace aruwlib
