#include "grabber_subsystem.hpp"
#include "squeeze_grabber_command.hpp"

// this is y axis command 

namespace aruwsrc
{

namespace engineer
{
    GrabberCommand::GrabberCommand(GrabberSubsystem* subsystem)
        : Command(), subsystemGrabber(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void GrabberCommand::initialize(void) 
    {
        subsystemGrabber->setSqueezed(true); // default movement is "squeezed"  
    }

    void GrabberCommand::execute(void) 
    {
        subsystemGrabber->setSqueezed(true);
    }
    
    void GrabberCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            subsystemGrabber->setSqueezed(false);
        }
        subsystemGrabber->setSqueezed(false);
    }

    bool GrabberCommand::isFinished(void) const
    {
        return false;
    }

    void GrabberCommand::interrupted(void) 
    {}
}  // namespace engineer

}  // namespace aruwsrc
