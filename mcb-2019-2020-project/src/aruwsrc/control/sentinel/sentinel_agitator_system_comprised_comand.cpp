#include "sentinel_agitator_system_comprised_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace sentinel
{

SentinelAgitatorSystemComprisedCommand::SentinelAgitatorSystemComprisedCommand(
    agitator::AgitatorSubsystem* agitator,
    agitator::AgitatorSubsystem* kicker,
    SentinelSwticherSubsystem* switcher) :
    agitator(agitator),
    kicker(kicker),
    switcher(switcher),
    rotateAgitator(agitator),
    rotateKicker(kicker, ROTATE_KICKER_ANGLE, 1, 0, false)
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(agitator));
    addSubsystemRequirement(dynamic_cast<Subsystem*>(kicker));
    addSubsystemRequirement(dynamic_cast<Subsystem*>(switcher));
    comprisedCommandScheduler.registerSubsystem(agitator);
    comprisedCommandScheduler.registerSubsystem(kicker);
    comprisedCommandScheduler.registerSubsystem(switcher);
}

void SentinelAgitatorSystemComprisedCommand::initialize()
{
    comprisedCommandScheduler.addCommand(&rotateAgitator);
    /// \todo this might be slightly delayed?
    comprisedCommandScheduler.addCommand(&rotateKicker);
    if (userLowerBarrel) {
        switcher->useLowerBarrel(userLowerBarrel);
    }
}

void SentinelAgitatorSystemComprisedCommand::execute()
{
    if (!comprisedCommandScheduler.isCommandScheduled(&rotateKicker))
    {
        comprisedCommandScheduler.addCommand(&rotateKicker);
    }
    comprisedCommandScheduler.run();
}

void SentinelAgitatorSystemComprisedCommand::end(bool interrupted)
{
    comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
    comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
}

bool SentinelAgitatorSystemComprisedCommand::isFinished() const
{
    return rotateAgitator.isFinished();
}

}  // namespace sentinel

}  // namespace aruwsrc
