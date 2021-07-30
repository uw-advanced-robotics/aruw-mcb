/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "analog_mock.hpp"
#include "can_mock.hpp"
#include "can_rx_handler_mock.hpp"
#include "can_rx_listener_mock.hpp"
#include "command_mapper_mock.hpp"
#include "command_mock.hpp"
#include "command_scheduler_mock.hpp"
#include "control_operator_interface_mock.hpp"
#include "digital_mock.hpp"
#include "dji_motor_mock.hpp"
#include "dji_motor_terminal_serial_handler_mock.hpp"
#include "dji_motor_tx_handler_mock.hpp"
#include "error_controller_mock.hpp"
#include "imu_rx_listener_mock.hpp"
#include "leds_mock.hpp"
#include "mpu6500_mock.hpp"
#include "pwm_mock.hpp"
#include "ref_serial_mock.hpp"
#include "remote_mock.hpp"
#include "scheduler_terminal_handler_mock.hpp"
#include "subsystem_mock.hpp"
#include "terminal_serial_mock.hpp"
#include "uart_mock.hpp"

// A file for listing all mock constructors and destructors since doing
// so in a source file allows for faster compilation than defining constructors
// in the headers
namespace tap::mock
{
AnalogMock::AnalogMock() {}
AnalogMock::~AnalogMock() {}

CanMock::CanMock() {}
CanMock::~CanMock() {}

CanRxListenerMock::CanRxListenerMock(tap::Drivers *drivers, uint32_t id, can::CanBus bus)
    : can::CanRxListener(drivers, id, bus)
{
}
CanRxListenerMock::~CanRxListenerMock() {}

CanRxHandlerMock::CanRxHandlerMock(tap::Drivers *drivers) : can::CanRxHandler(drivers) {}
CanRxHandlerMock::~CanRxHandlerMock() {}

CommandMock::CommandMock() : Command()
{
    // Most of the time tests expect that we are adding commands that
    // are ready to be added. This makes tests cleaner
    ON_CALL(*this, isReady).WillByDefault(testing::Return(true));
}
CommandMock::~CommandMock() {}

CommandMapperMock::CommandMapperMock(tap::Drivers *drivers) : control::CommandMapper(drivers) {}
CommandMapperMock::~CommandMapperMock() {}

ControlOperatorInterfaceMock::ControlOperatorInterfaceMock(tap::Drivers *drivers)
    : tap::control::ControlOperatorInterface(drivers)
{
}
ControlOperatorInterfaceMock::~ControlOperatorInterfaceMock() {}

CommandSchedulerMock::CommandSchedulerMock(tap::Drivers *drivers)
    : control::CommandScheduler(drivers)
{
}
CommandSchedulerMock::~CommandSchedulerMock() {}

DjiMotorMock::DjiMotorMock(
    Drivers *drivers,
    tap::motor::MotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char *name)
    : DjiMotor(drivers, desMotorIdentifier, motorCanBus, isInverted, name)
{
}
DjiMotorMock::~DjiMotorMock() {}

DigitalMock::DigitalMock() {}
DigitalMock::~DigitalMock() {}

DjiMotorTxHandlerMock::DjiMotorTxHandlerMock(tap::Drivers *drivers)
    : tap::motor::DjiMotorTxHandler(drivers)
{
}
DjiMotorTxHandlerMock::~DjiMotorTxHandlerMock() {}

DjiMotorTerminalSerialHandlerMock::DjiMotorTerminalSerialHandlerMock(tap::Drivers *drivers)
    : motor::DjiMotorTerminalSerialHandler(drivers)
{
}
DjiMotorTerminalSerialHandlerMock::~DjiMotorTerminalSerialHandlerMock() {}

ImuRxListenerMock::ImuRxListenerMock(Drivers *drivers) : ImuRxListener(drivers) {}
ImuRxListenerMock::~ImuRxListenerMock() {}

LedsMock::LedsMock() {}
LedsMock::~LedsMock() {}

ErrorControllerMock::ErrorControllerMock(tap::Drivers *drivers)
    : tap::errors::ErrorController(drivers)
{
}
ErrorControllerMock::~ErrorControllerMock() {}

Mpu6500Mock::Mpu6500Mock(tap::Drivers *drivers) : tap::sensors::Mpu6500(drivers) {}
Mpu6500Mock::~Mpu6500Mock() {}

PwmMock::PwmMock() {}
PwmMock::~PwmMock() {}

RefSerialMock::RefSerialMock(Drivers *drivers) : serial::RefSerial(drivers) {}
RefSerialMock::~RefSerialMock() {}

RemoteMock::RemoteMock(tap::Drivers *drivers) : tap::Remote(drivers) {}
RemoteMock::~RemoteMock() {}

SchedulerTerminalHandlerMock::SchedulerTerminalHandlerMock(Drivers *drivers)
    : control::SchedulerTerminalHandler(drivers)
{
}
SchedulerTerminalHandlerMock::~SchedulerTerminalHandlerMock() {}

SubsystemMock::SubsystemMock(Drivers *drivers) : control::Subsystem(drivers) {}
SubsystemMock::~SubsystemMock() {}

TerminalSerialMock::TerminalSerialMock(Drivers *drivers)
    : communication::serial::TerminalSerial(drivers)
{
}
TerminalSerialMock::~TerminalSerialMock() {}

UartMock::UartMock() {}
UartMock::~UartMock() {}

}  // namespace tap::mock
