#include "example_comprised_command.hpp"

namespace aruwsrc
{

namespace control
{

ExampleComprisedCommand::ExampleComprisedCommand(ExampleSubsystem* subsystem) :
Command(true),
exampleCommand(new ExampleCommand(subsystem))
{
    this->comprisedCommandScheduler.registerSubsystem(subsystem);
    this->addSubsystemRequirement(subsystem);
}

void ExampleComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(exampleCommand);
}

void ExampleComprisedCommand::execute() {
    this->comprisedCommandScheduler.runCommands();
}

void ExampleComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(exampleCommand, interrupted);
}

}  // namespace control

}  // namespace aruwsrc
