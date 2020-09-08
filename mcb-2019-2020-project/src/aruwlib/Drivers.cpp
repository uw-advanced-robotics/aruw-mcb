#include "Drivers.hpp"

namespace aruwlib
{
MockCan Drivers::can;
can::CanRxHandler Drivers::canRxHandler;
gpio::Analog Drivers::analog;
gpio::Digital Drivers::digital;
gpio::Leds Drivers::leds;
gpio::Pwm Drivers::pwm;
Remote Drivers::remote;
sensors::Mpu6500 Drivers::mpu6500;
serial::Uart Drivers::uart;
serial::XavierSerial Drivers::xavierSerial;
serial::RefSerial Drivers::refSerial;
control::CommandScheduler Drivers::commandScheduler;
control::ControlOperatorInterface Drivers::controlOperatorInterface;
control::CommandMapper Drivers::commandMapper;
errors::ErrorController Drivers::errorController;
DjiMotorTxHandlerMock Drivers::djiMotorTxHandler;

#ifdef ENV_SIMULATOR
void Drivers::reset() {}
#endif
}  // namespace aruwlib
