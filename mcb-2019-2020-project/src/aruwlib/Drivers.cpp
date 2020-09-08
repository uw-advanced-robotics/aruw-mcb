#include "Drivers.hpp"

namespace aruwlib
{
#ifndef ENV_SIMULATOR
can::Can Drivers::can;
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
motor::DjiMotorTxHandler Drivers::djiMotorTxHandler;
#else
CanMock Drivers::can;
CanRxHandlerMock Drivers::canRxHandler;
AnalogMock Drivers::analog;
DigitalMock Drivers::digital;
LedsMock Drivers::leds;
PwmMock Drivers::pwm;
Remote Drivers::remote;
Mpu6500Mock Drivers::mpu6500;
UartMock Drivers::uart;
XavierSerialMock Drivers::xavierSerial;
RefSerialMock Drivers::refSerial;
CommandSchedulerMock Drivers::commandScheduler;
ControlOperatorInterfaceMock Drivers::controlOperatorInterface;
CommandMapperMock Drivers::commandMapper;
ErrorControllerMock Drivers::errorController;
DjiMotorTxHandlerMock Drivers::djiMotorTxHandler;
#endif

#ifdef ENV_SIMULATOR
void Drivers::reset() {}
#endif
}  // namespace aruwlib
