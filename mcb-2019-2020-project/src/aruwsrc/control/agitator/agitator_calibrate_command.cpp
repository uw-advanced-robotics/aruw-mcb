#include "agitator_calibrate_command.hpp"
#include "src/aruwlib/control/subsystem.hpp"

namespace aruwsrc
{

namespace agitator
{
    AgitatorCalibrateCommand::AgitatorCalibrateCommand(AgitatorSubsystem* agitator) :
    agitator(agitator)
    {
        this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(agitator));
    }
    
    void AgitatorCalibrateCommand::initialize()
    {
        agitator->agitatorCalibrateHere();
    }

    void AgitatorCalibrateCommand::execute()
    {
        agitator->agitatorCalibrateHere();
    }

    void AgitatorCalibrateCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            agitator->agitatorCalibrateHere();
        }
    }

    bool AgitatorCalibrateCommand::isFinished() const
    {
        return agitator->agitatorCalibrateHere();
    }

    AgitatorSubsystem* agitator;

}  // namespace agitator

}  // namespace aruwsrc