/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/architecture/profiler.hpp"
#include "aruwlib/mock/analog_mock.hpp"
#include "aruwlib/mock/can_mock.hpp"
#include "aruwlib/mock/can_rx_handler_mock.hpp"
#include "aruwlib/mock/command_mapper_mock.hpp"
#include "aruwlib/mock/command_scheduler_mock.hpp"
#include "aruwlib/mock/control_operator_interface_mock.hpp"
#include "aruwlib/mock/digital_mock.hpp"
#include "aruwlib/mock/dji_motor_terminal_serial_handler_mock.hpp"
#include "aruwlib/mock/dji_motor_tx_handler_mock.hpp"
#include "aruwlib/mock/error_controller_mock.hpp"
#include "aruwlib/mock/imu_rx_listener_mock.hpp"
#include "aruwlib/mock/leds_mock.hpp"
#include "aruwlib/mock/mpu6500_mock.hpp"
#include "aruwlib/mock/pwm_mock.hpp"
#include "aruwlib/mock/ref_serial_mock.hpp"
#include "aruwlib/mock/remote_mock.hpp"
#include "aruwlib/mock/scheduler_terminal_handler_mock.hpp"
#include "aruwlib/mock/terminal_serial_mock.hpp"
#include "aruwlib/mock/uart_mock.hpp"

/* Start user mock includes */
#include "aruwsrc/mock/oled_display_mock.hpp"
#include "aruwsrc/mock/xavier_serial_mock.hpp"
/* End user mock includes */
#else
#include "aruwlib/architecture/profiler.hpp"
#include "aruwlib/communication/can/can.hpp"
#include "aruwlib/communication/can/can_rx_handler.hpp"
#include "aruwlib/communication/can/imu_rx_listener.hpp"
#include "aruwlib/communication/gpio/analog.hpp"
#include "aruwlib/communication/gpio/digital.hpp"
#include "aruwlib/communication/gpio/leds.hpp"
#include "aruwlib/communication/gpio/pwm.hpp"
#include "aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "aruwlib/communication/serial/ref_serial.hpp"
#include "aruwlib/communication/serial/remote.hpp"
#include "aruwlib/communication/serial/terminal_serial.hpp"
#include "aruwlib/communication/serial/uart.hpp"
#include "aruwlib/control/command_mapper.hpp"
#include "aruwlib/control/command_scheduler.hpp"
#include "aruwlib/control/control_operator_interface.hpp"
#include "aruwlib/control/scheduler_terminal_handler.hpp"
#include "aruwlib/errors/error_controller.hpp"
#include "aruwlib/motor/dji_motor_terminal_serial_handler.hpp"
#include "aruwlib/motor/dji_motor_tx_handler.hpp"

/* Start user mock includes */
#include "aruwsrc/communication/serial/xavier_serial.hpp"
#include "aruwsrc/display/oled_display.hpp"
/* End user mock includes */
#endif

namespace aruwlib
{
class Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : can(),
          canRxHandler(this),
          imuRxListener(this),
          analog(),
          digital(),
          leds(),
          pwm(),
          remote(this),
          mpu6500(this),
          uart(),
          refSerial(this),
          terminalSerial(this),
          djiMotorTerminalSerialHandler(this),
          schedulerTerminalHandler(this),
#ifdef ENV_UNIT_TESTS
          commandScheduler(this),
#else
          commandScheduler(this, true),
#endif
          controlOperatorInterface(this),
          commandMapper(this),
          errorController(this),
          djiMotorTxHandler(this),
          profiler(),
          /* Begin user constructor defines */
          xavierSerial(this),
          oledDisplay(this)
    /* End user mock drivers defines */ {}

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::CanMock> can;
    testing::NiceMock<mock::CanRxHandlerMock> canRxHandler;
    testing::NiceMock<mock::ImuRxListenerMock> imuRxListener;
    testing::NiceMock<mock::AnalogMock> analog;
    testing::NiceMock<mock::DigitalMock> digital;
    testing::NiceMock<mock::LedsMock> leds;
    testing::NiceMock<mock::PwmMock> pwm;
    testing::NiceMock<mock::RemoteMock> remote;
    testing::NiceMock<mock::Mpu6500Mock> mpu6500;
    testing::NiceMock<mock::UartMock> uart;
    testing::NiceMock<mock::RefSerialMock> refSerial;
    testing::NiceMock<mock::TerminalSerialMock> terminalSerial;
    testing::NiceMock<mock::DjiMotorTerminalSerialHandlerMock> djiMotorTerminalSerialHandler;
    testing::NiceMock<mock::SchedulerTerminalHandlerMock> schedulerTerminalHandler;
    testing::NiceMock<mock::CommandSchedulerMock> commandScheduler;
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::CommandMapperMock> commandMapper;
    mock::ErrorControllerMock errorController;
    testing::NiceMock<mock::DjiMotorTxHandlerMock> djiMotorTxHandler;
    arch::Profiler profiler;
    /* Begin user mock drivers defines */
    testing::NiceMock<aruwsrc::mock::XavierSerialMock> xavierSerial;
    testing::NiceMock<aruwsrc::mock::OledDisplayMock> oledDisplay;
/* End user mock drivers defines */
#else
public:
    can::Can can;
    can::CanRxHandler canRxHandler;
    can::ImuRxListener imuRxListener;
    gpio::Analog analog;
    gpio::Digital digital;
    gpio::Leds leds;
    gpio::Pwm pwm;
    Remote remote;
    sensors::Mpu6500 mpu6500;
    serial::Uart uart;
    serial::RefSerial refSerial;
    communication::serial::TerminalSerial terminalSerial;
    motor::DjiMotorTerminalSerialHandler djiMotorTerminalSerialHandler;
    control::SchedulerTerminalHandler schedulerTerminalHandler;
    control::CommandScheduler commandScheduler;
    control::ControlOperatorInterface controlOperatorInterface;
    control::CommandMapper commandMapper;
    errors::ErrorController errorController;
    motor::DjiMotorTxHandler djiMotorTxHandler;
    arch::Profiler profiler;
    /* Begin user drivers defines */
    aruwsrc::serial::XavierSerial xavierSerial;
    aruwsrc::display::OledDisplay oledDisplay;
/* End user drivers defines */
#endif
};  // class Drivers

}  // namespace aruwlib

#endif  // DRIVERS_HPP_