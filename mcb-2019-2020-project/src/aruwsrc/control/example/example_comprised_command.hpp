#ifndef __EXAMPLE_COMPRISED_COMAND_HPP__
#define __EXAMPLE_COMPRISED_COMAND_HPP__

#include <aruwlib/control/comprised_command.hpp>

#include "example_command.hpp"
#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
template <typename Drivers>
class ExampleComprisedCommand : public aruwlib::control::ComprisedCommand
{
public:
    explicit ExampleComprisedCommand(ExampleSubsystem<Drivers>* subsystem)
        : aruwlib::control::ComprisedCommand(),
          exampleCommand(subsystem, 2000),
          otherExampleCommand(subsystem, 500),
          switchTimer(2000),
          switchCommand(false)
    {
        this->addSubsystemRequirement(subsystem);
        this->comprisedCommandScheduler.registerSubsystem(subsystem);
    }

    void initialize() override
    {
        this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
    }

    void execute() override
    {
        if (switchTimer.execute())
        {
            switchTimer.restart(2000);
            if (switchCommand)
            {
                comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&otherExampleCommand));
            }
            else
            {
                comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
            }
            switchCommand = !switchCommand;
        }

        this->comprisedCommandScheduler.run();
    }

    void end(bool interrupted) override
    {
        comprisedCommandScheduler.removeCommand(
            dynamic_cast<Command*>(&exampleCommand),
            interrupted);
    }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "example comprised command"; }

private:
    ExampleCommand<Drivers> exampleCommand;

    ExampleCommand<Drivers> otherExampleCommand;

    aruwlib::arch::MilliTimeout switchTimer;

    bool switchCommand;
};

}  // namespace control

}  // namespace aruwsrc

#endif
