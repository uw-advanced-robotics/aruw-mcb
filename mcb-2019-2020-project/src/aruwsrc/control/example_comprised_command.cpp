#include "example_comprised_command.hpp"

namespace aruwsrc
{

namespace control
{

ExampleComprisedCommand::ExampleComprisedCommand(ExampleSubsystem* subsystem)
: exampleCommand(new ExampleCommand(subsystem))
{
    this->addSubsystemRequirement(subsystem);
}

void ExampleComprisedCommand::initialize()
{
    CommandScheduler::addCommand(exampleCommand);
}

void ExampleComprisedCommand::execute() {}

void ExampleComprisedCommand::end(bool interrupted)
{
    CommandScheduler::removeCommand(exampleCommand, interrupted);
}

}  // namespace control

}  // namespace aruwsrc
