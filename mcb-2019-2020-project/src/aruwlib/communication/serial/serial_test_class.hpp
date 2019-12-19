#ifndef __SERIAL_TEST_CLASS_HPP__
#define __SERIAL_TEST_CLASS_HPP__

#include "dji_serial.hpp"

namespace aruwlib
{

namespace serial
{

class SerialTestClass : DJISerial
{
 private:
    uint8_t messageId;

    uint8_t i;

 public:
    SerialTestClass();

    void messageReceiveCallback(SerialMessage completeMessage) override;

    void sendMessage(void);
};

}  // namespace serial

}  // namespace aruwlib

#endif
