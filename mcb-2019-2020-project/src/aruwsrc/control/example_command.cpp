#include "example_command.hpp"
#include "example_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    ExampleCommand::ExampleCommand(ExampleSubsystem* subsystem, int speed)
        : Command(), subsystemExample(subsystem), speed(speed)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    }

    void ExampleCommand::initialize()
    {}

    void ExampleCommand::execute()
    {
        subsystemExample->setDesiredRpm(speed);
    }

    void ExampleCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            subsystemExample->setDesiredRpm(0);
        }
        subsystemExample->setDesiredRpm(0);
    }

    bool ExampleCommand::isFinished(void) const
    {
        return false;
    }

    void ExampleCommand::interrupted(void)
    {}
}  // namespace control

}  // namespace aruwsrc
