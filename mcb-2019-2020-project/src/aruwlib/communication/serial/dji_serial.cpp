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
rxSequenceNumber(0),
CRC8(0),
CRC16(0),
handler(messageHandler),
lastTxMessageTimestamp(0),
lastRxMessageTimestamp(0)
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

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
bool DJISerial::periodicTask(SerialMessage* message) {
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
        }
    }
    return false;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void DJISerial::enableRxCRCEnforcement() {
    rxCRCEnforcementEnabled = true;
}
// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void DJISerial::disableRxCRCEnforcement() {
    rxCRCEnforcementEnabled = false;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
uint32_t DJISerial::getLastTxMessageTimestamp() {
    return this->lastTxMessageTimestamp;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
uint32_t DJISerial::getLastRxMessageTimestamp() {
    return this->lastRxMessageTimestamp;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
uint8_t DJISerial::getTxSequenceNumber() {
    return this->txSequenceNumber;
}

bool DJISerial::processFrameHeader() {
    // First Byte is always the head byte (0xA5)
    // Skipping First Byte
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
    // Check if the messageLength is small than size of buffer
    if (currentExpectedMessageLength == 0 ||
    currentExpectedMessageLength >=
    SERIAL_RX_BUFF_SIZE - (FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH)) {
        return false;
    }

    if(rxCRCEnforcementEnabled) {
        if(verifyCRC8(rxBuffer, FRAME_HEADER_LENGTH - 1, CRC8)) {
            return processFrameData();
        }
    } else {
        return processFrameData();
    }

    currentExpectedMessageLength = 0;
    return false;
}

bool DJISerial::processFrameData() {
    // get the rest of the packet (1-byte sequence, 1-byte CRC8, 2-byte
    // message type, message data, 2-byte CRC16)
    // skip frame header
    uint32_t actualLength = this->read((rxBuffer + FRAME_HEADER_LENGTH),
        currentExpectedMessageLength + FRAME_TYPE_LENGTH + FRAME_CRC16_LENGTH);

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

    if (rxCRCEnforcementEnabled) {
        if (!verifyCRC16(rxBuffer, FRAME_HEADER_LENGTH + FRAME_TYPE_LENGTH +
                currentExpectedMessageLength, CRC16)) {
            // CRC checking is enabled but CRC8/CRC16 were invalid
            currentExpectedMessageLength = 0;
            // NON-FATAL-ERROR-CHECK
            return false;
        }
    }

    // CRC checking is enabled and CRC8/CRC16 were valid
    SerialMessage message;
    message.length = currentExpectedMessageLength;
    message.data = rxBuffer + FRAME_DATA_OFFSET;
    message.type = currentMessageType;
    this->lastRxMessage = message;
    lastRxMessageTimestamp = modm::Clock::now().getTime();
    handler(&message);
    return true;
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
