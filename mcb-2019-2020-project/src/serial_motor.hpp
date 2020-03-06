#ifndef SERIAL_MOTOR_HPP
#define SERIAL_MOTOR_HPP

namespace src
{
namespace logger
{

class SerialMotor
{
    public:

    // Prints the motors and corresponding info
    static void printMotors();

    // Prints individual motor and corresponding info
    static void printMotor();

    // Keep printing individual motor and relevant info
    static void pollData();

    private: 
    // Retrieves the motors from the dji_motor
    // Stores them in a field?
    // Can call the field later to print
    static void getMotors();

    
};

} // namespace logger

} // namespace src

#endif