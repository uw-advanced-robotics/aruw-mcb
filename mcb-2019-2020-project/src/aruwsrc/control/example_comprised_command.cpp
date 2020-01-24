#include "example_comprised_command.hpp"

#include "example_command.hpp"
#include "example_subsystem.hpp"


namespace aruwsrc
{

namespace control
{

ExampleComprisedCommand::ExampleComprisedCommand(ExampleSubsystem* subsystem) :
Command(true),
exampleCommand(subsystem),
otherExampleCommand(subsystem),
switchTimer(2000)
{
    this->comprisedCommandScheduler.registerSubsystem(subsystem);
    this->addSubsystemRequirement(subsystem);
}

void ExampleComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
}

void ExampleComprisedCommand::execute() {
    if (switchTimer.execute()) {
        switchTimer.restart(2000);
        if (this->comprisedCommandScheduler.isCommandScheduled(dynamic_cast<Command*>(&exampleCommand)))
        {
            this->comprisedCommandScheduler.removeCommand(dynamic_cast<Command*>(&exampleCommand), false);
            this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&otherExampleCommand));
        }
        else
        {
            this->comprisedCommandScheduler.removeCommand(dynamic_cast<Command*>(&otherExampleCommand), false);
            this->comprisedCommandScheduler.addCommand(dynamic_cast<Command*>(&exampleCommand));
        }
    }

    this->comprisedCommandScheduler.run();
}

void ExampleComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(dynamic_cast<Command*>(&exampleCommand), interrupted);
}

}  // namespace control

}  // namespace aruwsrc
