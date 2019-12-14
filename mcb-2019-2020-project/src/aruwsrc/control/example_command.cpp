#include "example_command.hpp"
#include "example_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    ExampleCommand::ExampleCommand(ExampleSubsystem* subsystem)
        : Command(), subsystemExample(subsystem)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    }

    void ExampleCommand::initialize()
    {}

    void ExampleCommand::execute()
    {
        subsystemExample->setDesiredRpm(DEFAULT_WHEEL_RPM);
    }

    void ExampleCommand::end(bool interrupted)
    {}

    bool ExampleCommand::isFinished(void) const
    {
        return false;
    }
}  // namespace control

}  // namespace aruwsrc
