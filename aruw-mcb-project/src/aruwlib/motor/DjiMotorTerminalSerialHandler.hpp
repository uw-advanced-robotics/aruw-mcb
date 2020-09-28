/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DJI_MOTOR_TERMINAL_SERIAL_HANDLER_HPP_
#define DJI_MOTOR_TERMINAL_SERIAL_HANDLER_HPP_

#include "aruwlib/communication/serial/TerminalSerial.hpp"

#include "dji_motor_tx_handler.hpp"

namespace aruwlib
{
namespace motor
{
class DjiMotorTxHandler;
class DjiMotorTerminalSerialHandler : public communication::serial::ITerminalSerialCallback
{
public:
    DjiMotorTerminalSerialHandler(DjiMotorTxHandler* motorHandler) : motorHandler(motorHandler) {}

    bool terminalSerialCallback(std::stringstream&& inputLine, modm::IOStream& outputStream)
        override;

private:
    typedef DjiMotor const* (DjiMotorTxHandler::*getMotorByIdFunc)(MotorId);

    void getMotorInfoToString(const DjiMotor& motor, modm::IOStream& outputStream);

    void printAllMotorInfo(getMotorByIdFunc func, modm::IOStream& outputStream);

    DjiMotorTxHandler* motorHandler;
};  // class DjiMotorTerminalSerialHandler
}  // namespace motor
}  // namespace aruwlib

#endif  // DJI_MOTOR_TERMINAL_SERIAL_HANDLER_HPP_
