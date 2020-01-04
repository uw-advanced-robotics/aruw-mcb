#include "shoot_comprised_command.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

ShootComprisedCommand::ShootComprisedCommand(
    AgitatorSubsystem* agitator,
    float agitatorChangeAngle,
    float maxUnjamAngle
    ) : connectedAgitator(agitator),
    agitatorRotateCommand(new AgitatorRotateCommand(agitator, agitatorChangeAngle)),
    agitatorUnjamCommand(new AgitatorUnjamCommand(agitator, maxUnjamAngle)),
    unjamSequenceCommencing(false)
{
    this->addSubsystemRequirement(reinterpret_cast<Subsystem*>(agitator));
}

void ShootComprisedCommand::initialize()
{
    CommandScheduler::smrtPtrCommandCast(agitatorRotateCommand)->initialize();
    unjamSequenceCommencing = false;
}

void ShootComprisedCommand::execute()
{
    if (connectedAgitator->isAgitatorJammed() && !unjamSequenceCommencing)
    {
        // when the agitator is jammed, add the agitatorUnjamCommand
        // the to scheduler. The rotate forward command will be automatically
        // unscheduled.
        unjamSequenceCommencing = true;
        CommandScheduler::smrtPtrCommandCast(agitatorRotateCommand)->end(true);
        CommandScheduler::smrtPtrCommandCast(agitatorUnjamCommand)->initialize();
    }

    if (unjamSequenceCommencing)
    {
        CommandScheduler::smrtPtrCommandCast(agitatorUnjamCommand)->execute();
    }
    else
    {
        CommandScheduler::smrtPtrCommandCast(agitatorRotateCommand)->execute();
    }
}

void ShootComprisedCommand::end(bool interrupted)
{
    if (unjamSequenceCommencing)
    {
        CommandScheduler::smrtPtrCommandCast(agitatorUnjamCommand)->end(interrupted);
    }
    else
    {
        CommandScheduler::smrtPtrCommandCast(agitatorRotateCommand)->end(interrupted);
    }
}

bool ShootComprisedCommand::isFinished() const
{
    return (CommandScheduler::smrtPtrCommandCast(agitatorRotateCommand)->isFinished()
        && !unjamSequenceCommencing)
        || (CommandScheduler::smrtPtrCommandCast(agitatorUnjamCommand)->isFinished()
        && unjamSequenceCommencing);
}

}  // namespace control

}  // namespace aruwsrc
