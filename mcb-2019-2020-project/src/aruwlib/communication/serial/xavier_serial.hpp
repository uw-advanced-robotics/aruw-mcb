#ifndef __SERIAL_XAVIER_HPP__
#define __SERIAL_XAVIER_HPP__

#include "dji_serial.hpp"

namespace aruwlib
{

namespace serial
{

class XavierSerial : DJISerial
{
 private:
    uint8_t messageId;

    uint8_t i;

 public:
    XavierSerial();

    void messageReceiveCallback(SerialMessage_t completeMessage) override;

    void sendMessage(void);
};

}  // namespace serial

}  // namespace aruwlib

#endif
