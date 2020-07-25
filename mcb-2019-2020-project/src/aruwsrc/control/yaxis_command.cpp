#include "yaxis_command.hpp"
#include "yaxis_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    YAxisCommand::YAxisCommand(YAxisSubsystem* subsystem, YAxisSubsystem::Position positionYAxis)
        : subsystemYAxis(subsystem), positionYAxis(positionYAxis)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void YAxisCommand::initialize()
    {}

    void YAxisCommand::execute()
    {
        subsystemYAxis->setPosition(positionYAxis); 
    }

    void YAxisCommand::end(bool interrupted)
    {
        end(interrupted);
    }

    bool YAxisCommand::isFinished() const
    {
        return false;
    }

    void YAxisCommand::interrupted()
    {}
}

}