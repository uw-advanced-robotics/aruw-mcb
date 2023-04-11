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

#ifndef VIRTUAL_MCB_HANDLER_HPP_
#define VIRTUAL_MCB_HANDLER_HPP_

#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "can/virtual_can_bus.hpp"
#include "can/virtual_can_rx_handler.hpp"
#include "motor/virtual_dji_motor_tx_handler.hpp"

namespace aruwsrc::virtualMCB
{
template <tap::communication::serial::Uart::UartPort port>
class VirtualMCBHandler
{
public:
    VirtualMCBHandler(tap::Drivers* drivers);

    void refresh();

    VirtualCanBus<port> canbus;
    VirtualDJIMotorTxHandler motorTxHandler;
    VirtualCANRxHandler canRxHandler;
};

// This is essentially a forward declaration but needed for template class to build,
// https://stackoverflow.com/a/37189280
template class VirtualMCBHandler<tap::communication::serial::Uart::UartPort::Uart1>;
template class VirtualMCBHandler<tap::communication::serial::Uart::UartPort::Uart2>;
template class VirtualMCBHandler<tap::communication::serial::Uart::UartPort::Uart3>;
template class VirtualMCBHandler<tap::communication::serial::Uart::UartPort::Uart6>;
template class VirtualMCBHandler<tap::communication::serial::Uart::UartPort::Uart7>;
template class VirtualMCBHandler<tap::communication::serial::Uart::UartPort::Uart8>;

}  // namespace aruwsrc::virtualMCB

#endif