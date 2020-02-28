#include "xaxis_command.hpp"
#include "xaxis_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    XAxisCommand::XAxisCommand(XAxisSubsystem* subsystem)
        : Command(), subsystemXAxis(subsystem), positionXAxis(positionXAxis)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void XAxisCommand::initialize()
    {}

    void XAxisCommand::execute(XAxisSubsystem::Position)
    {
        subsystemXAxis->setPosition(positionXAxis); 
    }

    void XAxisCommand::end(bool interrupted)
    {
        end(interrupted);
        // if (interrupted)
        // {
        //     subsystemXAxis->setPosition(currentPosition);
        // }
        // subsystemXAxis->setPosition(currentPosition);
    }

    bool XAxisCommand::isFinished() const
    {
        return false;
    }

    void XAxisCommand::interrupted()
    {}
}

}