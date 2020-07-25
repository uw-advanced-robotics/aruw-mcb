#include "xaxis_command.hpp"

namespace aruwsrc
{
namespace engineer
{
    XAxisCommand::XAxisCommand(XAxisSubsystem* subsystem, XAxisSubsystem::Position positionYAxis)
        : subsystemYAxis(subsystem), positionYAxis(positionYAxis)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void XAxisCommand::initialize()
    {}

    void XAxisCommand::execute()
    {
        subsystemYAxis->setPosition(positionYAxis); 
    }

    void XAxisCommand::end(bool interrupted)
    {}

    bool XAxisCommand::isFinished() const
    {
        return false;
    }
}  // namespace engineer
}  // namespace control
