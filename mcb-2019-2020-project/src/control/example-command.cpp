#include "src/control/example-command.hpp"
#include "src/control/example-subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    ExampleCommand::ExampleCommand(ExampleSubsystem* subsystem)
        : Command(false), subsystemExample(subsystem)
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
    {
        if (interrupted)
        {}
        subsystemExample->setDesiredRpm(0);
    }

    bool ExampleCommand::isFinished(void)
    {
        return false;
    }

    void ExampleCommand::interrupted(void)
    {}
}  // namespace control

}  // namespace aruwsrc
