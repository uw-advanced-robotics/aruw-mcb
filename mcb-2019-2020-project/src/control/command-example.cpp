#include "src/control/command-example.hpp"
#include "src/control/subsystem-example.hpp"

namespace aruwsrc
{

namespace control
{
    CommandExample::CommandExample(SubsystemExample* subsystem)
        : Command(false), subsystemExample(subsystem)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    }

    void CommandExample::initialize()
    {}

    void CommandExample::execute()
    {
        subsystemExample->setDesiredRpm(DEFAULT_WHEEL_RPM);
    }

    void CommandExample::end(bool interrupted)
    {
        if (interrupted)
        {
            subsystemExample->setDesiredRpm(0);
        }
    }

int count = 0;
    bool CommandExample::isFinished(void)
    {
        return ++count == 5;
        return false;
    }

    void CommandExample::interrupted(void)
    {
    }
}  // namespace control

}  // namespace aruwsrc
