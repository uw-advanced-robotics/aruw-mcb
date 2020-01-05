#ifndef __HPP__
#define __HPP__

#include "src/aruwlib/control/comprised_command.hpp"
#include "example_command.hpp"

namespace aruwsrc
{

namespace control
{

class ExampleComprisedCommand : public ComprisedCommand
{
 public:
    ExampleComprisedCommand(ExampleSubsystem* subsystem) : exampleCommand(new ExampleCommand(subsystem))
    {
        this->addSubsystemRequirement(subsystem);
    }

    void initialize()
    {
        CommandScheduler::addCommand(exampleCommand);
    }

    void execute()
    {}

    void end(bool interrupted)
    {}

 private:
    modm::SmartPointer exampleCommand;
};

}

}

#endif
