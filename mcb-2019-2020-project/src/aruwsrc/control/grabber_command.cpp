#include "src/aruwsrc/control/grabber_command.hpp"
#include "src/aruwsrc/control/grabber_subsystem.hpp"

// this is y axis command 

namespace aruwsrc
{

namespace control
{
    GrabberCommand::GrabberCommand(GrabberSubsystem* subsystem)
        : Command(), subsystemGrabber(subsystem)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    }

    void GrabberCommand::initialize(void) 
    {
        subsystemGrabber->setMovement(0); // default movement is "not extended"  
    }

    void GrabberCommand::execute(void) 
    {
        subsystemGrabber->setMovement(true);
    }
    
    void GrabberCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            subsystemGrabber->setMovement(false);
        }
        subsystemGrabber->setMovement(false);
    }

    bool GrabberCommand::isFinished(void) const
    {
        return false;
    }

    void GrabberCommand::interrupted(void) 
    {}
}  // namespace control

}  // namespace aruwsrc
