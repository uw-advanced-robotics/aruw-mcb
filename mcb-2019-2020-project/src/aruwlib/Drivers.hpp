#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#ifndef ENV_SIMULATOR
#include "communication/can/can.hpp"
#include "communication/can/can_rx_handler.hpp"
#include "communication/gpio/analog.hpp"
#include "communication/gpio/digital.hpp"
#include "communication/gpio/leds.hpp"
#include "communication/gpio/pwm.hpp"
#include "communication/remote.hpp"
#include "communication/sensors/mpu6500/mpu6500.hpp"
#include "communication/serial/ref_serial.hpp"
#include "communication/serial/uart.hpp"
#include "communication/serial/xavier_serial.hpp"
#include "control/command_mapper.hpp"
#include "control/command_scheduler.hpp"
#include "control/control_operator_interface.hpp"
#include "errors/error_controller.hpp"
#include "motor/dji_motor_tx_handler.hpp"
#else
#include <gmock/gmock.h>

#include "mocks/AnalogMock.hpp"
#include "mocks/CanMock.hpp"
#include "mocks/CanRxHandlerMock.hpp"
#include "mocks/CommandMapperMock.hpp"
#include "mocks/CommandSchedulerMock.hpp"
#include "mocks/ControlOperatorInterfaceMock.hpp"
#include "mocks/DigitalMock.hpp"
#include "mocks/DjiMotorTxHandlerMock.hpp"
#include "mocks/ErrorControllerMock.hpp"
#include "mocks/LedsMock.hpp"
#include "mocks/Mpu6500Mock.hpp"
#include "mocks/PwmMock.hpp"
#include "mocks/RefSerialMock.hpp"
#include "mocks/RemoteMock.hpp"
#include "mocks/UartMock.hpp"
#include "mocks/XavierSerialMock.hpp"
#endif

namespace aruwlib
{
class Drivers
{
public:
#ifndef ENV_SIMULATOR
    static can::Can can;
    static can::CanRxHandler canRxHandler;
    static gpio::Analog analog;
    static gpio::Digital digital;
    static gpio::Leds leds;
    static gpio::Pwm pwm;
    static Remote remote;
    static sensors::Mpu6500 mpu6500;
    static serial::Uart uart;
    static serial::XavierSerial xavierSerial;
    static serial::RefSerial refSerial;
    static control::CommandScheduler commandScheduler;
    static control::ControlOperatorInterface controlOperatorInterface;
    static control::CommandMapper commandMapper;
    static errors::ErrorController errorController;
    static motor::DjiMotorTxHandler djiMotorTxHandler;
#else
    static void reset();

    static CanMock can;
    static CanRxHandlerMock canRxHandler;
    static AnalogMock analog;
    static DigitalMock digital;
    static LedsMock leds;
    static PwmMock pwm;
    static Remote remote;
    static Mpu6500Mock mpu6500;
    static UartMock uart;
    static XavierSerialMock xavierSerial;
    static RefSerialMock refSerial;
    static CommandSchedulerMock commandScheduler;
    static ControlOperatorInterfaceMock controlOperatorInterface;
    static CommandMapperMock commandMapper;
    static ErrorControllerMock errorController;
    static DjiMotorTxHandlerMock djiMotorTxHandler;
#endif
};  // class Drivers
}  // namespace aruwlib

#endif  // DRIVERS_HPP_
