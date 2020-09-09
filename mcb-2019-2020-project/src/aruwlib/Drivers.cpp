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

void Drivers::initialize()
{
    testing::Mock::AllowLeak(&aruwlib::Drivers::analog);
    testing::Mock::AllowLeak(&aruwlib::Drivers::can);
    testing::Mock::AllowLeak(&aruwlib::Drivers::canRxHandler);
    testing::Mock::AllowLeak(&aruwlib::Drivers::analog);
    testing::Mock::AllowLeak(&aruwlib::Drivers::digital);
    testing::Mock::AllowLeak(&aruwlib::Drivers::leds);
    testing::Mock::AllowLeak(&aruwlib::Drivers::pwm);
    testing::Mock::AllowLeak(&aruwlib::Drivers::remote);
    testing::Mock::AllowLeak(&aruwlib::Drivers::mpu6500);
    testing::Mock::AllowLeak(&aruwlib::Drivers::uart);
    testing::Mock::AllowLeak(&aruwlib::Drivers::xavierSerial);
    testing::Mock::AllowLeak(&aruwlib::Drivers::refSerial);
    testing::Mock::AllowLeak(&aruwlib::Drivers::commandScheduler);
    testing::Mock::AllowLeak(&aruwlib::Drivers::controlOperatorInterface);
    testing::Mock::AllowLeak(&aruwlib::Drivers::commandMapper);
    testing::Mock::AllowLeak(&aruwlib::Drivers::errorController);
    testing::Mock::AllowLeak(&aruwlib::Drivers::djiMotorTxHandler);
}
void Drivers::reset()
{
    testing::Mock::VerifyAndClearExpectations(&can);
    testing::Mock::VerifyAndClearExpectations(&canRxHandler);
    testing::Mock::VerifyAndClearExpectations(&analog);
    testing::Mock::VerifyAndClearExpectations(&digital);
    testing::Mock::VerifyAndClearExpectations(&leds);
    testing::Mock::VerifyAndClearExpectations(&pwm);
    testing::Mock::VerifyAndClearExpectations(&remote);
    testing::Mock::VerifyAndClearExpectations(&mpu6500);
    testing::Mock::VerifyAndClearExpectations(&uart);
    testing::Mock::VerifyAndClearExpectations(&xavierSerial);
    testing::Mock::VerifyAndClearExpectations(&refSerial);
    testing::Mock::VerifyAndClearExpectations(&commandScheduler);
    testing::Mock::VerifyAndClearExpectations(&controlOperatorInterface);
    testing::Mock::VerifyAndClearExpectations(&commandMapper);
    testing::Mock::VerifyAndClearExpectations(&errorController);
    testing::Mock::VerifyAndClearExpectations(&djiMotorTxHandler);
}
#endif
}  // namespace aruwlib
