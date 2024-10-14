#include "sweep_command.hpp"

#include "tap/architecture/clock.hpp"

namespace aruwsrc::control::sweep
{

void SweepCommand::initialize(){
    currentFrequency = startFrequency;
    startTime = tap::arch::clock::getTimeMilliseconds();
    commandRunning = false;
}

void SweepCommand::execute(){
    commandRunning = true;
    
    timeDifference = tap::arch::clock::getTimeMilliseconds() - startTime;

    setpoint = sin(currentFrequency * (timeDifference / 1000.0));

    currentFrequency += frequencySweepRate;
    subsystem->motor->setDesiredOutput(setpoint * tap::motor::DjiMotor::MAX_OUTPUT_GM6020);
}

}  // namespace aruwsrc::control::sweep
