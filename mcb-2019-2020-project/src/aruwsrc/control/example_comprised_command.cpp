#include "example_comprised_command.hpp"

namespace aruwsrc
{

namespace control
{

ExampleComprisedCommand::ExampleComprisedCommand(ExampleSubsystem* subsystem) :
Command(true),
exampleCommand(subsystem)
{
    this->comprisedCommandScheduler.registerSubsystem(subsystem);
    this->addSubsystemRequirement(subsystem);
}

void ExampleComprisedCommand::initialize()
{
    this->comprisedCommandScheduler.addCommand(reinterpret_cast<Command*>(exampleCommand));
}

void ExampleComprisedCommand::execute() {
    this->comprisedCommandScheduler.run();
}

void ExampleComprisedCommand::end(bool interrupted)
{
    this->comprisedCommandScheduler.removeCommand(reinterpret_cast<Command*>(exampleCommand), interrupted);
}

}  // namespace control

}  // namespace aruwsrc
