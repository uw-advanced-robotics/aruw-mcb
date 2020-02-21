#include "xaxis_command.hpp"
#include "xaxis_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    XAxisCommand::XAxisCommand(XAxisSubsystem* subsystem)
        : Command(), subsystemXAxis(subsystem)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    }

    void XAxisCommand::initialize()
    {}

    void XAxisCommand::execute()
    {
        subsystemXAxis->setPosition(desiredPosition);
    }

    void XAxisCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            subsystemXAxis->setPosition(currentPosition);
        }
        subsystemXAxis->setPosition(currentPosition);
    }

    bool XAxisCommand::isFinished(void) const
    {
        return false;
    }

    void XAxisCommand::interrupted(void)
    {}
}

}