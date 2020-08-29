#ifndef __SERIAL_TEST_CLASS_HPP__
#define __SERIAL_TEST_CLASS_HPP__

#include "dji_serial.hpp"

namespace aruwlib
{
namespace serial
{
/**
 * A simple serial tester to insure `DjiSerial` is working properly.
 *
 * @note to test with a single MCB, instantiate a test class and connect
 *      TX to RX. You should be able to check if messages are being received.
 *      Additionally, watch `i` to check if you are dropping messages.
 */
template <typename Drivers> class SerialTestClass : public DJISerial<Drivers>
{
public:
    ///< Attaches this test class to `Uart2`.
    SerialTestClass() : DJISerial<Drivers>(Uart::UartPort::Uart2, true), messageId(0), i(0) {}

    ///< Stores the sequenceNumber in `messageId`.
    void messageReceiveCallback(const SerialMessage& completeMessage) override
    {
        messageId = completeMessage.sequenceNumber;
    }

    ///< Sends a message of length 1, the byte `60`, with the `sequenceNumber` incremented.
    void sendMessage()
    {
        this->txMessage.length = 1;
        this->txMessage.headByte = 0xa5;
        this->txMessage.sequenceNumber = i;
        this->txMessage.type = 4;
        this->txMessage.data[0] = 60;
        this->send();
        i++;
    }

private:
    uint8_t messageId;

    uint8_t i;
};

}  // namespace serial

}  // namespace aruwlib

#endif
