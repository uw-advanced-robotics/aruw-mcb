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
constexpr char DjiMotorTerminalSerialHandler::HEADER[];
constexpr char DjiMotorTerminalSerialHandler::USAGE[];

void DjiMotorTerminalSerialHandler::terminalSerialStreamCallback(modm::IOStream& outputStream)
{
    printInfo(outputStream);
}

bool DjiMotorTerminalSerialHandler::terminalSerialCallback(
    std::stringstream&& inputLine,
    modm::IOStream& outputStream,
    bool)
{
    std::string arg;
    motorId = 0;
    canBusValid = false;
    canBus = 0;
    printAll = false;
    while (inputLine >> arg)
    {
        if (arg == "motor")
        {
            if (!(inputLine >> motorId) ||
                motorId < (DJI_MOTOR_NORMALIZED_ID(MotorId::MOTOR1) + 1) ||
                motorId > (DJI_MOTOR_NORMALIZED_ID(MotorId::MOTOR8) + 1))
            {
                outputStream << USAGE;
                return false;
            }
            motorId--;
            motorIdValid = true;
        }
        else if (arg == "can")
        {
            if (!(inputLine >> canBus) || (canBus != 0 && canBus != 1))
            {
                outputStream << USAGE;
                return false;
            }
            canBusValid = true;
        }
        else if (arg == "all")
        {
            printAll = true;
            break;
        }
        else if (arg == "-H")
        {
            outputStream << USAGE;
            return true;
        }
        else
        {
            outputStream << USAGE;
            return false;
        }
    }

    return printInfo(outputStream);
}

bool DjiMotorTerminalSerialHandler::printInfo(modm::IOStream& outputStream)
{
    if (printAll)
    {
        outputStream << "CAN 1:" << modm::endl;
        printAllMotorInfo(&DjiMotorTxHandler::getCan1MotorData, outputStream);
        outputStream.flush();
        outputStream << "CAN 2:" << modm::endl;
        printAllMotorInfo(&DjiMotorTxHandler::getCan2MotorData, outputStream);
    }
    else if (!canBusValid && !motorIdValid)
    {
        outputStream << USAGE;
        return false;
    }
    else if (canBusValid && !motorIdValid)
    {
        if (canBus == 1)
        {
            printAllMotorInfo(&DjiMotorTxHandler::getCan1MotorData, outputStream);
        }
        else if (canBus == 2)
        {
            printAllMotorInfo(&DjiMotorTxHandler::getCan2MotorData, outputStream);
        }
        else
        {
            outputStream << USAGE;
            return false;
        }
    }
    else if (!canBusValid && motorIdValid)
    {
        outputStream << "CAN 1:" << modm::endl;
        getMotorInfoToString(
            motorHandler->getCan1MotorData(static_cast<MotorId>(motorId + aruwlib::motor::MOTOR1)),
            outputStream);
        outputStream << "CAN 2:" << modm::endl;
        getMotorInfoToString(
            motorHandler->getCan2MotorData(static_cast<MotorId>(motorId + aruwlib::motor::MOTOR1)),
            outputStream);
    }
    else
    {
        if (canBus == 1)
        {
            getMotorInfoToString(
                motorHandler->getCan1MotorData(
                    static_cast<MotorId>(motorId + aruwlib::motor::MOTOR1)),
                outputStream);
        }
        else
        {
            getMotorInfoToString(
                motorHandler->getCan2MotorData(
                    static_cast<MotorId>(motorId + aruwlib::motor::MOTOR1)),
                outputStream);
        }
    }
    return true;
}

void DjiMotorTerminalSerialHandler::getMotorInfoToString(
    const DjiMotor* motor,
    modm::IOStream& outputStream)
{
    if (motor != nullptr)
    {
        outputStream << (DJI_MOTOR_NORMALIZED_ID(motor->getMotorIdentifier()) + 1) << ". "
                     << motor->getName() << ": online: " << (motor->isMotorOnline() ? "yes" : "no")
                     << ", enc wrapped: " << motor->encStore.getEncoderWrapped()
                     << ", rpm: " << motor->getShaftRPM() << modm::endl;
    }
}

void DjiMotorTerminalSerialHandler::printAllMotorInfo(
    getMotorByIdFunc func,
    modm::IOStream& outputStream)
{
    for (int i = static_cast<int>(MOTOR1); i <= static_cast<int>(MOTOR8); i++)
    {
        const DjiMotor* motor = (motorHandler->*(func))(static_cast<MotorId>(i));
        getMotorInfoToString(motor, outputStream);
    }
}
}  // namespace motor
}  // namespace aruwlib
