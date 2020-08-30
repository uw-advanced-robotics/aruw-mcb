#ifndef __SHOOT_COMPRISED_COMMAND_HPP__
#define __SHOOT_COMPRISED_COMMAND_HPP__

#include <aruwlib/control/comprised_command.hpp>

#include "agitator_rotate_command.hpp"
#include "agitator_subsystem.hpp"
#include "agitator_unjam_command.hpp"

namespace aruwsrc
{
namespace agitator
{
template <typename Drivers>
class ShootComprisedCommand : public aruwlib::control::ComprisedCommand<Drivers>
{
public:
    ShootComprisedCommand(
        AgitatorSubsystem<Drivers>* agitator,
        float agitatorChangeAngle,
        float maxUnjamAngle,
        uint32_t agitatorDesiredRotateTime,
        uint32_t minAgitatorRotateTime)
        : connectedAgitator(agitator),
          agitatorRotateCommand(
              agitator,
              agitatorChangeAngle,
              agitatorDesiredRotateTime,
              false,
              minAgitatorRotateTime),
          agitatorUnjamCommand(agitator, maxUnjamAngle),
          unjamSequenceCommencing(false)
    {
        this->comprisedCommandScheduler.registerSubsystem(agitator);
        this->addSubsystemRequirement(
            dynamic_cast<aruwlib::control::Subsystem<Drivers>*>(agitator));
    }

    void initialize() override
    {
        this->comprisedCommandScheduler.addCommand(
            dynamic_cast<aruwlib::control::Command<Drivers>*>(&agitatorRotateCommand));
        unjamSequenceCommencing = false;
    }

    void execute() override
    {
        if (connectedAgitator->isAgitatorJammed() && !unjamSequenceCommencing)
        {
            // when the agitator is jammed, add the agitatorUnjamCommand
            // the to scheduler. The rotate forward command will be automatically
            // unscheduled.
            unjamSequenceCommencing = true;
            this->comprisedCommandScheduler.addCommand(
                dynamic_cast<aruwlib::control::Command<Drivers>*>(&agitatorUnjamCommand));
        }
        this->comprisedCommandScheduler.run();
    }

    void end(bool interrupted) override
    {
        this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<aruwlib::control::Command<Drivers>*>(&agitatorUnjamCommand),
            interrupted);
        this->comprisedCommandScheduler.removeCommand(
            dynamic_cast<aruwlib::control::Command<Drivers>*>(&agitatorRotateCommand),
            interrupted);
    }

    bool isFinished() const override
    {
        return (agitatorRotateCommand.isFinished() && !unjamSequenceCommencing) ||
               (agitatorUnjamCommand.isFinished() && unjamSequenceCommencing);
    }

    const char* getName() const override { return "agitator shoot command"; }

private:
    AgitatorSubsystem<Drivers>* connectedAgitator;

    AgitatorRotateCommand<Drivers> agitatorRotateCommand;

    AgitatorUnjamCommand<Drivers> agitatorUnjamCommand;

    bool unjamSequenceCommencing;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif
