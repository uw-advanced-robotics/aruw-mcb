#ifndef SERIAL_MOTOR_HPP
#define SERIAL_MOTOR_HPP

#include "serial_data_logger.hpp"
#include "dji_motor.hpp"

namespace src
{
namespace logger
{

class SerialMotor
{
    
    public:
    // Prints individual motor and corresponding info
    void printMotor();

    // Prints the motors and corresponding info
    void printMotors();

    // Keep printing individual motor and relevant info
    void pollData();

    private: 
    // Retrieves the motors from the dji_motor
    // Stores them in a field?
    // Can call the field later to print
    void getMotors();
};

} // namespace logger

} // namespace src

#endif