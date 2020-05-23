#include "uart_bno055.hpp"

namespace aruwlib {

namespace sensors {

template < class Uart >
void UartBno055<Uart>::read() {
    if (setOperationModeTimeout.execute()) {
        setOperationMode(operationMode);
        setOperationModeTimeout.restart();
        requestDataIntervalTimeout.restart();
    }

    if (disconnectTimeout.execute()) {
        connected = false;
        clearRxBuffer();
    }

    if (requestDataIntervalTimeout.execute()) {
        requestRegisterData(Bno055Enum::Register::ACCEL_DATA_X_LSB, Bno055Data::size);
        requestDataIntervalTimeout.restart();
    }

    uint8_t data;
    while (Uart::read(data) && currentBufferIndex < expectedReadLength) {
        connected = true;
        switch (data) {
            case DATA_RESPONSE_START_BYTE: {
                rxState = SerialRxState::WAIT_FOR_DATA_LENGTH;
                currentBufferIndex = 0;
                expectedReadLength = 2;
                break;
            }

            case STATUS_RESPONSE_START_BYTE: {
                rxState = SerialRxState::READ_ACK;
                currentBufferIndex = 0;
                expectedReadLength = 2;
                break;
            }

            default: {
                if (rxState == SerialRxState::WAIT_FOR_DATA_LENGTH) {
                    expectedReadLength = data;
                    if (expectedReadLength > RX_BUFFER_LENGTH) {
                        rxState = SerialRxState::WAIT_FOR_HEADER;
                        currentBufferIndex = 0;
                        expectedReadLength = 1;
                    } else {
                        rxState = SerialRxState::READ_DATA;
                    }
                }
                break;
            }
        }
        rxBuffer[currentBufferIndex] = data;
        currentBufferIndex += 1;
        communicationTimeout.restart();
        disconnectTimeout.restart();
        lastReceivedDataTimestamp = modm::Clock::now();
    }

    if (communicationTimeout.execute()) {
        rxState = SerialRxState::WAIT_FOR_HEADER;
        currentBufferIndex = 0;
        expectedReadLength = 1;
    }

    if (currentBufferIndex >= expectedReadLength) {
        switch (rxState) {
            case SerialRxState::READ_DATA: {
                handleDataResponse();
                clearRxBuffer();
                break;
            }
            case SerialRxState::READ_ACK : {
                handleStatusResponse();
                clearRxBuffer();
                break;
            }
            default : {
                expectedReadLength = 1;
                clearRxBuffer();
                break;
            }
        }
        rxState = SerialRxState::WAIT_FOR_HEADER;
    }
}

template < class Uart >
void UartBno055<Uart>::initialize() {
    reset();
}

template < class Uart >
void UartBno055<Uart>::reset() {
    updateRegister(Bno055Enum::Register::SYS_TRIGGER,
            static_cast<uint8_t>(Bno055Enum::SystemTrigger::RST_SYS));
}

template < class Uart >
bool UartBno055<Uart>::setOperationMode(Bno055Enum::OperationMode mode) {
    return updateRegister(Bno055Enum::Register::OPR_MODE, static_cast<uint8_t>(mode));
}

template < class Uart >
bool UartBno055<Uart>::requestRegisterData(Bno055Enum::Register reg, uint8_t length) {
    ReadCommand command;
    command.registerAddress = reg;
    command.length = length;
    return sendCommand(&command);
}

template < class Uart >
bool UartBno055<Uart>::updateRegister(Bno055Enum::Register reg, uint8_t mask) {
    WriteCommand command;
    command.registerAddress = reg;
    command.length = 1;
    uint8_t data = mask;
    command.data = &data;
    return sendCommand(&command);
}

template < class Uart >
bool UartBno055<Uart>::sendCommand(WriteCommand* command) {
    uint8_t* buff = static_cast<uint8_t*>(malloc(command->size()));
    buff[0] = command->startByte;
    buff[1] = command->readWriteMode;
    buff[2] = static_cast<uint8_t>(command->registerAddress);
    buff[3] = command->length;
    memcpy(buff + 4, command->data, command->length);
    bool result = Uart::write(buff, command->size());
    free(buff);
    return result;
}

template < class Uart >
bool UartBno055<Uart>::sendCommand(ReadCommand* command) {
    return Uart::write(reinterpret_cast<uint8_t*>(command), sizeof(command));
}

template < class Uart >
void UartBno055<Uart>::clearRxBuffer() {
    currentBufferIndex = 0;
    memset(rxBuffer, 0, RX_BUFFER_LENGTH);
}

template < class Uart >
void UartBno055<Uart>::handleDataResponse() {
    DataResponse response;
    response.length = rxBuffer[1];
    response.data = rxBuffer + 2;
    memcpy(&imuData.raw, response.data, response.length);
    lastResponseStatus = ResponseStatus::NO_RESPONSE;
}

template < class Uart >
void UartBno055<Uart>::handleStatusResponse() {
    ResponseStatus status = static_cast<ResponseStatus>(rxBuffer[1]);
    lastResponseStatus = status;
    switch (status)
    {
    case ResponseStatus::WRONG_START_BYTE: {
        Uart::discardTransmitBuffer();
        Uart::discardReceiveBuffer();
        requestDataIntervalTimeout.restart();
        break;
    }
    case ResponseStatus::BUS_OVER_RUN_ERROR: {
        Uart::discardTransmitBuffer();
        Uart::discardReceiveBuffer();
        requestDataIntervalTimeout.restart();
        break;
    }
    default:
        break;
    }
}
#ifndef ENV_SIMULATOR
template class UartBno055<Usart1>;
template class UartBno055<Usart2>;
template class UartBno055<Usart3>;
template class UartBno055<Usart6>;
template class UartBno055<Uart7>;
template class UartBno055<Uart8>;
#endif

}  // namespace sensors

}  // namespace aruwlib
