#include "serial_motor.hpp"

namespace src
{
namespace logger
{
// Pass in a motor/string? 
// maybe need some sort of conversion method
// using the retrieved data and then turning them
// into strings? 
void logger::SerialMotor::printMotor() {
    // using the temp storage from getMotors()
    // index the specific motor based on
    // user input of: get <motor_name>
    // prints <motor_name> and all the relevant data
}

void logger::SerialMotor::printMotors() {
    // using the temp storage from getMotors()
    // call this method when user input matches: get all motors
    // prints all <motor_names> and all their corresponding data
}

void logger::SerialMotor::pollData() {
    // continuosly call printMotor()?
}

void logger::SerialMotor::getMotors() {
    djigetCanBus()
    // use Darsh's get motor method
    // save it into a temp storage
    // get the motor name too?
}
  
} // namespace logger

} // namespace src