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

    std::string terminalSerialCallback(std::stringstream&& inputLine) override;

private:
    typedef DjiMotor const* (DjiMotorTxHandler::*getMotorByIdFunc)(MotorId);

    std::string getMotorInfoToString(const DjiMotor& motor);

    void printAllMotorInfo(getMotorByIdFunc func, std::string& outStr);

    DjiMotorTxHandler* motorHandler;
};  // class DjiMotorTerminalSerialHandler
}  // namespace motor
}  // namespace aruwlib

#endif  // DJI_MOTOR_TERMINAL_SERIAL_HANDLER_HPP_
