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

#include "DjiMotorTerminalSerialHandler.hpp"

#include "dji_motor_tx_handler.hpp"

namespace aruwlib
{
namespace motor
{
bool DjiMotorTerminalSerialHandler::terminalSerialCallback(
    std::stringstream&& inputLine,
    modm::IOStream& outputStream)
{
    std::string arg;
    MotorId motorId = MotorId::MOTOR1;
    can::CanBus canBus = can::CanBus::CAN_BUS1;
    bool motorIdValid = false;
    bool canBusValid = false;
    bool printAllMotors = false;

    while (inputLine >> arg)
    {
        if (arg == "all")
        {
            printAllMotors = true;
        }
        else if (arg == "motor1")
        {
            motorId = MotorId::MOTOR1;
            motorIdValid = true;
        }
        else if (arg == "motor2")
        {
            motorId = MotorId::MOTOR2;
            motorIdValid = true;
        }
        else if (arg == "motor3")
        {
            motorId = MotorId::MOTOR3;
            motorIdValid = true;
        }
        else if (arg == "motor4")
        {
            motorId = MotorId::MOTOR4;
            motorIdValid = true;
        }
        else if (arg == "motor5")
        {
            motorId = MotorId::MOTOR5;
            motorIdValid = true;
        }
        else if (arg == "motor6")
        {
            motorId = MotorId::MOTOR6;
            motorIdValid = true;
        }
        else if (arg == "motor7")
        {
            motorId = MotorId::MOTOR7;
            motorIdValid = true;
        }
        else if (arg == "motor8")
        {
            motorId = MotorId::MOTOR8;
            motorIdValid = true;
        }
        else if (arg == "can1")
        {
            canBus = can::CanBus::CAN_BUS1;
            canBusValid = true;
        }
        else if (arg == "can2")
        {
            canBus = can::CanBus::CAN_BUS2;
            canBusValid = true;
        }
        else
        {
            return false;
        }
    }

    if (printAllMotors)
    {
        outputStream << "CAN 1:" << modm::endl;
        printAllMotorInfo(&DjiMotorTxHandler::getCan1MotorData, outputStream);
        outputStream << "CAN 2:" << modm::endl;
        printAllMotorInfo(&DjiMotorTxHandler::getCan2MotorData, outputStream);
        return true;
    }
    else if (canBusValid && motorIdValid)
    {
        switch (canBus)
        {
            case can::CanBus::CAN_BUS1:
                if (motorHandler->getCan1MotorData(motorId) == nullptr)
                {
                    outputStream << "motor no in tx handler" << modm::endl;
                }
                else
                {
                    getMotorInfoToString(*motorHandler->getCan1MotorData(motorId), outputStream);
                }
                break;
            case can::CanBus::CAN_BUS2:
                if (motorHandler->getCan2MotorData(motorId) == nullptr)
                {
                    outputStream << "motor not in tx handler" << modm::endl;
                }
                else
                {
                    getMotorInfoToString(*motorHandler->getCan2MotorData(motorId), outputStream);
                }
                break;
        }
        return true;
    }
    else
    {
        outputStream << "bad args" << modm::endl;
        return false;
    }
}

void DjiMotorTerminalSerialHandler::getMotorInfoToString(
    const DjiMotor& motor,
    modm::IOStream& outputStream)
{
    outputStream << (DJI_MOTOR_NORMALIZED_ID(motor.getMotorIdentifier()) + 1) << ". "
                 << motor.getName() << ": online: " << (motor.isMotorOnline() ? "yes" : "no")
                 << ", enc wrapped: " << motor.encStore.getEncoderWrapped()
                 << ", rpm: " << motor.getShaftRPM() << modm::endl;
}

void DjiMotorTerminalSerialHandler::printAllMotorInfo(
    getMotorByIdFunc func,
    modm::IOStream& outputStream)
{
    for (int i = static_cast<int>(MOTOR1); i <= static_cast<int>(MOTOR8); i++)
    {
        const DjiMotor* motor = (motorHandler->*(func))(static_cast<MotorId>(i));
        if (motor != nullptr)
        {
            getMotorInfoToString(*motor, outputStream);
        }
    }
}
}  // namespace motor
}  // namespace aruwlib
