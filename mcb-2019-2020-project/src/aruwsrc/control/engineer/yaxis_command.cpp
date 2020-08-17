#include "yaxis_command.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>

namespace aruwsrc
{
namespace engineer
{
YAxisCommand::YAxisCommand(YAxisSubsystem* subsystem, YAxisSubsystem::Position positionYAxis)
    : subsystemYAxis(subsystem),
      positionYAxis(positionYAxis)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
}

void YAxisCommand::initialize() {}

void YAxisCommand::execute() { subsystemYAxis->setPosition(positionYAxis); }

void YAxisCommand::end(bool interrupted)
{
    // If interrupted, it is likely that the user stopped the mechanism for some reason, so stop
    // attempting to reach the target. Otherwise, the mechanism should presumably continue on to
    // the final setpoint.
    if (interrupted)
    {
        subsystemYAxis->stop();
    }
}

bool YAxisCommand::isFinished() const
{
    return aruwlib::algorithms::compareFloatClose(
        subsystemYAxis->getCurrentPosition(),
        subsystemYAxis->getDesiredPosition(),
        COMPLETE_DIFF_CRITERIA);
}
}  // namespace engineer
}  // namespace aruwsrc
