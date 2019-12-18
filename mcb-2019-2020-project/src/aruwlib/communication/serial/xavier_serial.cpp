#include "xavier_serial.hpp"

namespace aruwlib
{

namespace serial
{

XavierSerial::XavierSerial():
DJISerial(DJISerial::SerialPort::PORT_UART6, false)
{}

void XavierSerial::messageReceiveCallback(SerialMessage_t completeMessage)
{
}

void XavierSerial::sendMessage()
{
}

}  // namespace serial

}  // namespace aruwlib
