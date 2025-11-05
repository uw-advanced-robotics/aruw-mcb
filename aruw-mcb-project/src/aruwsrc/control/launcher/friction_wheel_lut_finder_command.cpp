
#include "friction_wheel_lut_finder_command.hpp"

#include "friction_wheel_subsystem.hpp"

namespace aruwsrc::control::launcher
{
    FrictionWheelLUTFinderCommand::FrictionWheelLUTFinderCommand(FrictionWheelSubsystem *subsystem)
        : subsystem(subsystem)
    {
        this->addSubsystemRequirement(subsystem);
    }

    void FrictionWheelLUTFinderCommand::initialize() 
    {
        prevTime = tap::arch::clock::getTimeMilliseconds();
    }

    void FrictionWheelLUTFinderCommand::execute()
    {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        if (currTime - prevTime > TIME_INC_MILLI) {
            curRPM += RPM_INCREMENT;
            this->subsystem->setDesiredRPM(curRPM);
            prevTime = currTime; 
        }
    }

    void FrictionWheelLUTFinderCommand::end(bool) { this->subsystem->setDesiredRPM(0.0); }

    bool FrictionWheelLUTFinderCommand::isFinished() const { return curRPM >= RPM_MAX; }

}