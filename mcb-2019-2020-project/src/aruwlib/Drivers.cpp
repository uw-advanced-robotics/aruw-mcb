#include "Drivers.hpp"

namespace aruwlib
{
#define RESET_DRIVER(className, ClassName) (className = ClassName())

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

#ifdef ENV_SIMULATOR
void Drivers::reset()
{
    RESET_DRIVER(can, can::Can);
    RESET_DRIVER(canRxHandler, can::CanRxHandler);
    RESET_DRIVER(analog, gpio::Analog);
    RESET_DRIVER(digital, gpio::Digital);
    RESET_DRIVER(leds, gpio::Leds);
    RESET_DRIVER(pwm, gpio::Pwm);
    RESET_DRIVER(remote, Remote);
    RESET_DRIVER(uart, serial::Uart);
    RESET_DRIVER(xavierSerial, serial::XavierSerial);
    RESET_DRIVER(refSerial, serial::RefSerial);
    RESET_DRIVER(commandScheduler, control::CommandScheduler);
    RESET_DRIVER(controlOperatorInterface, control::ControlOperatorInterface);
    RESET_DRIVER(commandMapper, control::CommandMapper);
    RESET_DRIVER(errorController, errors::ErrorController);
    RESET_DRIVER(djiMotorTxHandler, motor::DjiMotorTxHandler);
}
#endif
}  // namespace aruwlib
