#include "DjiMotorTerminalSerialHandler.hpp"

#include "dji_motor_tx_handler.hpp"

namespace aruwlib
{
namespace motor
{
std::string DjiMotorTerminalSerialHandler::terminalSerialCallback(std::stringstream&& inputLine)
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
            return "invalid arguments";
        }
    }
    if (printAllMotors)
    {
        std::string allMotors;
        allMotors += "CAN 1:\n";
        printAllMotorInfo(&DjiMotorTxHandler::getCan1MotorData, allMotors);
        allMotors += "CAN 2:\n";
        printAllMotorInfo(&DjiMotorTxHandler::getCan2MotorData, allMotors);
        return allMotors;
    }
    else if (canBusValid && motorIdValid)
    {
        switch (canBus)
        {
            case can::CanBus::CAN_BUS1:
                if (motorHandler->getCan1MotorData(motorId) == nullptr)
                {
                    return "motor not in tx handler";
                }
                return getMotorInfoToString(*motorHandler->getCan1MotorData(motorId));
            case can::CanBus::CAN_BUS2:
                if (motorHandler->getCan2MotorData(motorId) == nullptr)
                {
                    return "motor not in tx handler";
                }
                return getMotorInfoToString(*motorHandler->getCan2MotorData(motorId));
            default:
                return "invalid arguments";
        }
    }
    else
    {
        return "invalid agruments";
    }
}

std::string DjiMotorTerminalSerialHandler::getMotorInfoToString(const DjiMotor& motor)
{
    return motor.getMotorIdentifier() + ". " + motor.getName() +
           ": online: " + (motor.isMotorOnline() ? "yes" : "no") +
           ", enc wrapped: " + std::to_string(motor.encStore.getEncoderWrapped()) +
           ", rpm: " + std::to_string(motor.getShaftRPM()) + "\n";
}

void DjiMotorTerminalSerialHandler::printAllMotorInfo(getMotorByIdFunc func, std::string& outStr)
{
    for (int i = static_cast<int>(MOTOR1); i <= static_cast<int>(MOTOR8); i++)
    {
        const DjiMotor* motor = (motorHandler->*(func))(static_cast<MotorId>(i));
        if (motor != nullptr)
        {
            outStr += std::to_string(i) + ") " + getMotorInfoToString(*motor);
        }
    }
}
}  // namespace motor
}  // namespace aruwlib
