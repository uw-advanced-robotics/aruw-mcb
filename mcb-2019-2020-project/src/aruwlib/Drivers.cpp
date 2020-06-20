#include "Drivers.hpp"

namespace aruwlib
{
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
    resetCan();
    resetCanRxHandler();
    resetAnalog();
    resetDigital();
    resetLeds();
    resetPwm();
    resetRemote();
    resetMpu6500();
    resetUart();
    resetXavierSerial();
    resetRefSerial();
    resetCommandScheduler();
    resetControlOperatorInterface();
    resetCommandMapper();
    resetErrorController();
    resetDjiMotorTxHandler();
}

void Drivers::resetCan()
{
    can::Can tmp;
    can = tmp;
}

void Drivers::resetCanRxHandler()
{
    can::CanRxHandler tmp;
    canRxHandler = tmp;
}

void Drivers::resetAnalog()
{
    gpio::Analog tmp;
    analog = tmp;
}

void Drivers::resetDigital()
{
    gpio::Digital tmp;
    digital = tmp;
}

void Drivers::resetLeds()
{
    gpio::Leds tmp;
    leds = tmp;
}

void Drivers::resetPwm()
{
    gpio::Pwm tmp;
    pwm = tmp;
}

void Drivers::resetRemote()
{
    Remote tmp;
    remote = tmp;
}

void Drivers::resetMpu6500()
{
    sensors::Mpu6500 tmp;
    mpu6500 = tmp;
}

void Drivers::resetUart()
{
    serial::Uart tmp;
    uart = tmp;
}

void Drivers::resetXavierSerial()
{
    serial::XavierSerial tmp;
    xavierSerial = tmp;
}

void Drivers::resetRefSerial()
{
    serial::RefSerial tmp;
    refSerial = tmp;
}

void Drivers::resetCommandScheduler()
{
    control::CommandScheduler tmp;
    commandScheduler = tmp;
}

void Drivers::resetControlOperatorInterface()
{
    control::ControlOperatorInterface tmp;
    controlOperatorInterface = tmp;
}

void Drivers::resetCommandMapper()
{
    control::CommandMapper tmp;
    commandMapper = tmp;
}

void Drivers::resetErrorController()
{
    errors::ErrorController tmp;
    errorController = tmp;
}

void Drivers::resetDjiMotorTxHandler()
{
    motor::DjiMotorTxHandler tmp;
    djiMotorTxHandler = tmp;
}

#endif
}  // namespace aruwlib
