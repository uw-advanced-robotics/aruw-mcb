#include "wrist_calibrate_command.hpp"
#include "src/aruwlib/control/subsystem.hpp"

namespace aruwsrc
{

namespace engineer
{
    WristCalibrateCommand::WristCalibrateCommand(WristSubsystem* wrist) :
        wrist(wrist)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(wrist));
    }

    void WristCalibrateCommand::initialize()
    {
        wrist->wristCalibrateHere();
    }

    void WristCalibrateCommand::execute()
    {
        wrist->wristCalibrateHere();
    }

    void WristCalibrateCommand::end(bool interrupted)
    {}

    bool WristCalibrateCommand::isFinished() const
    {
        return wrist->wristCalibrateHere();
    }

    WristSubsystem* wrist;

}  // namespace engineer

}  // namespace aruwsrc