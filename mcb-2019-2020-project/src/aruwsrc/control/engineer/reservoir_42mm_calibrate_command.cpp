#include "reservoir_42mm_calibrate_command.hpp"

namespace aruwsrc
{

namespace engineer
{

    Reservoir42mmCalibrateCommand::Reservoir42mmCalibrateCommand(Reservoir42mmSubsystem* reservoir) :
        reservoir(reservoir)
    {
        this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(reservoir));
    }

    void Reservoir42mmCalibrateCommand::initialize()
    {
        reservoir->reservoirCalibrateHere();
    }

    void Reservoir42mmCalibrateCommand::execute()
    {
        reservoir->reservoirCalibrateHere();
    }

    void Reservoir42mmCalibrateCommand::end(bool interrupted)
    {}

    bool Reservoir42mmCalibrateCommand::isFinished() const
    {
        return reservoir->reservoirCalibrateHere();
    }

    Reservoir42mmSubsystem* reservoir;

}  // namespace engineer

}  // namespace aruwsrc