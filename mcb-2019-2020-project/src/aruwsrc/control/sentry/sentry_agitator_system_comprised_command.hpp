#ifndef __SENTRY_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP__
#define __SENTRY_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP__

#include "src/aruwlib/control/comprised_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_shoot_comprised_command_instances.hpp"
#include "src/aruwsrc/control/agitator/agitator_rotate_command.hpp"

namespace aruwsrc
{

namespace sentry
{

class SentryAgitatorSystemComprisedCommand : public aruwlib::control::ComprisedCommand
{
 public:
    SentryAgitatorSystemComprisedCommand(agitator::AgitatorSubsystem* agitator,
                                         agitator::AgitatorSubsystem* kicker);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

 private:
    static constexpr float ROTATE_KICKER_ANGLE = 2 * aruwlib::algorithms::PI;
    agitator::AgitatorSubsystem* agitator;

    agitator::AgitatorSubsystem* kicker;

    agitator::ShootFastComprisedCommand rotateAgitator;

    agitator::AgitatorRotateCommand rotateKicker;
};

}  // namespace sentry

}  // namespace aruwsrc

#endif
