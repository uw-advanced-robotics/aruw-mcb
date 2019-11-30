#include "shoot_comprised_command.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"
#include "src/control/scheduler.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

ShootComprisedCommand::ShootComprisedCommand(
    AgitatorSubsystem* agitator,
    float agitatorChangeAngle,
    float maxUnjamAngle,
    uint32_t maxUnjamWaitTime,
    float unjamSetpointTolerance
    ) : connectedAgitator(agitator),
    agitatorRotateCommand(new AgitatorRotateCommand(agitator, agitatorChangeAngle)),
    agitatorUnjamCommand(new AgitatorUnjamCommand(agitator, maxUnjamAngle, maxUnjamWaitTime, unjamSetpointTolerance)),
    unjamSequenceCommencing(false)
{}

void ShootComprisedCommand::initialize()
{
    CommandScheduler::getCmdPtr(agitatorRotateCommand)->initialize();
}

void ShootComprisedCommand::execute()
{
    if (connectedAgitator->isAgitatorJammed() && !unjamSequenceCommencing)
    {
        // when the agitator is jammed, add the agitatorUnjamCommand
        // the to scheduler. The rotate forward command will be automatically
        // unscheduled.
        unjamSequenceCommencing = true;
        CommandScheduler::getCmdPtr(agitatorRotateCommand)->end(true);
        CommandScheduler::getCmdPtr(agitatorUnjamCommand)->initialize();
    }

    if (unjamSequenceCommencing)
    {
        CommandScheduler::getCmdPtr(agitatorUnjamCommand)->execute();
    }
    else
    {
        CommandScheduler::getCmdPtr(agitatorRotateCommand)->execute();
    }
}

void ShootComprisedCommand::end(bool interrupted)
{
    if (unjamSequenceCommencing)
    {
        CommandScheduler::getCmdPtr(agitatorUnjamCommand)->end(interrupted);
    }
    else
    {
        CommandScheduler::getCmdPtr(agitatorRotateCommand)->end(interrupted);
    }
}

bool ShootComprisedCommand::isFinished() const
{
    return (CommandScheduler::getCmdPtr(agitatorRotateCommand)->isFinished()
        && !unjamSequenceCommencing)
        || (CommandScheduler::getCmdPtr(agitatorUnjamCommand)->isFinished()
        && unjamSequenceCommencing);
}

}  // namespace control

}  // namespace aruwsrc
