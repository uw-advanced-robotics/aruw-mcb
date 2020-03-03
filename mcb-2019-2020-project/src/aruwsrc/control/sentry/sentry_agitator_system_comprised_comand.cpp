#include "sentry_agitator_system_comprised_command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace sentry
{

SentryAgitatorSystemComprisedCommand::SentryAgitatorSystemComprisedCommand(
    agitator::AgitatorSubsystem* agitator,
    agitator::AgitatorSubsystem* kicker,
    SentrySwticherSubsystem* switcher) :
    agitator(agitator),
    kicker(kicker),
    switcher(switcher),
    rotateAgitator(agitator),
    rotateKicker(kicker, ROTATE_KICKER_ANGLE, 1, 0, false)
{
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(agitator));
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(kicker));
    this->addSubsystemRequirement(dynamic_cast<Subsystem*>(switcher));
    this->comprisedCommandScheduler.registerSubsystem(agitator);
    this->comprisedCommandScheduler.registerSubsystem(kicker);
    this->comprisedCommandScheduler.registerSubsystem(switcher);
}

void SentryAgitatorSystemComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(&rotateAgitator);
    /// \todo this might be slightly delayed?
    this->comprisedCommandScheduler.addCommand(&rotateKicker);
    if (userLowerBarrel) {
        switcher->useLowerBarrel(userLowerBarrel);
    }
}

void SentryAgitatorSystemComprisedCommand::execute()
{
    if (!this->comprisedCommandScheduler.isCommandScheduled(&rotateKicker))
    {
        this->comprisedCommandScheduler.addCommand(&rotateKicker);
    }
    this->comprisedCommandScheduler.run();
}

void SentryAgitatorSystemComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
    this->comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
}

bool SentryAgitatorSystemComprisedCommand::isFinished() const
{
    return rotateAgitator.isFinished();
}
    
}  // namespace sentry

}  // namespace aruwsrc
