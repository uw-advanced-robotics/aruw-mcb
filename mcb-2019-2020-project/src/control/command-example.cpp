#include "src/control/command-example.hpp"
#include "src/control/subsystem-example.hpp"

namespace aruwsrc
{

namespace control
{
    void CommandExample::initialize()
    {}
    
    void CommandExample::execute()
    {
        subsystemExample->setDesiredRpm(DEFAULT_WHEEL_RPM);
    }

    void CommandExample::end(bool interrupted)
    {
        if (!interrupted)
        {
            subsystemExample->setDesiredRpm(0);
        }
    }
    
    bool CommandExample::isFinished(void)
    {
        return false;
    }

    void CommandExample::interrupted(void)
    {
        end(true);
    }

    bool CommandExample::runsWhenDisabled(void) const
    {
        return false;
    }
}  // namespace control

}  // namespace aruwsrc
