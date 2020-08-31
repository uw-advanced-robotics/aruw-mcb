#include "HALDrivers.hpp"

namespace aruwlib
{
can::Can HALDrivers::can;
can::CanRxHandler<HALDrivers> HALDrivers::canRxHandler;
gpio::Analog HALDrivers::analog;
gpio::Digital HALDrivers::digital;
gpio::Leds HALDrivers::leds;
gpio::Pwm HALDrivers::pwm;
Remote<HALDrivers> HALDrivers::remote;
sensors::Mpu6500<HALDrivers> HALDrivers::mpu6500;
serial::Uart HALDrivers::uart;
serial::XavierSerial<HALDrivers> HALDrivers::xavierSerial;
serial::RefSerial<HALDrivers> HALDrivers::refSerial;
control::CommandScheduler<HALDrivers> HALDrivers::commandScheduler;
control::ControlOperatorInterface<HALDrivers> HALDrivers::controlOperatorInterface;
control::CommandMapper<HALDrivers> HALDrivers::commandMapper;
errors::ErrorController<HALDrivers> HALDrivers::errorController;
motor::DjiMotorTxHandler<HALDrivers> HALDrivers::djiMotorTxHandler;
}  // namespace aruwlib
