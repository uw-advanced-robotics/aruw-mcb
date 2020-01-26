#include "shoot_steady_comprised_command.hpp"
#include "agitator_rotate_command.hpp"
#include "agitator_unjam_command.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace agitator
{

ShootSteadyComprisedCommand::ShootSteadyComprisedCommand(
    AgitatorSubsystem* agitator,
    float agitatorChangeAngle,
    float maxUnjamAngle,
    float agitatorDesiredRotateTime,
    float minAgitatorRotateTime) :
    connectedAgitator(agitator),
    agitatorRotateCommand(
        agitator,
        agitatorChangeAngle,
        agitatorDesiredRotateTime,
        minAgitatorRotateTime
    ),
    agitatorUnjamCommand(agitator, maxUnjamAngle),
    unjamSequenceCommencing(false)
{
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(agitator));
}

void ShootSteadyComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&agitatorRotateCommand));
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
        this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&agitatorRotateCommand), true);
        this->comprisedCommandScheduler.addCommand(
            dynamic_cast<Command*>(&agitatorUnjamCommand));
    }
}

void ShootSteadyComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorUnjamCommand), interrupted);
    this->comprisedCommandScheduler.removeCommand(
        dynamic_cast<Command*>(&agitatorRotateCommand), interrupted);
}

bool ShootSteadyComprisedCommand::isFinished() const
{
    return (agitatorRotateCommand.isFinished()
        && !unjamSequenceCommencing)
        || (agitatorUnjamCommand.isFinished()
        && unjamSequenceCommencing);
}

}  // namespace control

}  // namespace aruwsrc
