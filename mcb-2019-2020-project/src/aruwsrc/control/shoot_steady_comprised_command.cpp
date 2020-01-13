#include "shoot_steady_comprised_command.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

ShootSteadyComprisedCommand::ShootSteadyComprisedCommand(
    AgitatorSubsystem* agitator,
    float agitatorChangeAngle,
    float maxUnjamAngle
    ) : connectedAgitator(agitator),
    agitatorRotateCommand(new AgitatorRotateCommand(agitator, agitatorChangeAngle)),
    agitatorUnjamCommand(new AgitatorUnjamCommand(agitator, maxUnjamAngle)),
    unjamSequenceCommencing(false)
{
    addUseCommand(agitatorRotateCommand);
    addUseCommand(agitatorUnjamCommand);
    this->addSubsystemRequirement(reinterpret_cast<Subsystem*>(agitator));
}

void ShootSteadyComprisedCommand::initialize()
{
    CommandScheduler::addCommand(agitatorRotateCommand);
    unjamSequenceCommencing = false;
}

void ShootSteadyComprisedCommand::execute()
{
    if (connectedAgitator->isAgitatorJammed() && !unjamSequenceCommencing)
    {
        // when the agitator is jammed, add the agitatorUnjamCommand
        // the to scheduler. The rotate forward command will be automatically
        // unscheduled.
        unjamSequenceCommencing = true;
        CommandScheduler::removeCommand(agitatorRotateCommand, true);
        CommandScheduler::addCommand(agitatorUnjamCommand);
    }
}

void ShootSteadyComprisedCommand::end(bool interrupted)
{
    CommandScheduler::removeCommand(agitatorUnjamCommand, interrupted);
    CommandScheduler::removeCommand(agitatorRotateCommand, interrupted);
}

bool ShootSteadyComprisedCommand::isFinished() const
{
    return (CommandScheduler::smrtPtrCommandCast(agitatorRotateCommand)->isFinished()
        && !unjamSequenceCommencing)
        || (CommandScheduler::smrtPtrCommandCast(agitatorUnjamCommand)->isFinished()
        && unjamSequenceCommencing);
}

}  // namespace control

}  // namespace aruwsrc
