#ifndef __HPP__
#define __HPP__

#include "example_command.hpp"
#include "example_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class ExampleComprisedCommand : public Command
{
 public:
    explicit ExampleComprisedCommand(ExampleSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const override
    {
       return false;
    }

 private:
    modm::SmartPointer exampleCommand;
};

}  // namespace control

}  // namespace aruwsrc

#endif
