#ifndef __HPP__
#define __HPP__

#include "src/aruwlib/control/comprised_command.hpp"
#include "example_command.hpp"
#include "example_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class ExampleComprisedCommand : public ComprisedCommand
{
 public:
    explicit ExampleComprisedCommand(ExampleSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool interrupted);

 private:
    modm::SmartPointer exampleCommand;
};

}  // namespace control

}  // namespace aruwsrc

#endif
