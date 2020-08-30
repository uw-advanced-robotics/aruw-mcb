#include "Drivers.hpp"

namespace aruwlib
{
can::Can Drivers::can;
can::CanRxHandler<Drivers> Drivers::canRxHandler;
gpio::Analog Drivers::analog;
gpio::Digital Drivers::digital;
gpio::Leds Drivers::leds;
gpio::Pwm Drivers::pwm;
Remote<Drivers> Drivers::remote;
sensors::Mpu6500<Drivers> Drivers::mpu6500;
serial::Uart Drivers::uart;
serial::XavierSerial<Drivers> Drivers::xavierSerial;
serial::RefSerial<Drivers> Drivers::refSerial;
control::CommandScheduler<Drivers> Drivers::commandScheduler;
control::ControlOperatorInterface<Drivers> Drivers::controlOperatorInterface;
control::CommandMapper<Drivers> Drivers::commandMapper;
errors::ErrorController<Drivers> Drivers::errorController;
motor::DjiMotorTxHandler<Drivers> Drivers::djiMotorTxHandler;
}  // namespace aruwlib
