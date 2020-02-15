#include "wrist_calibrate_command.hpp"
#include "src/aruwlib/control/subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    WristCalibrateCommand::WristCalibrateCommand(WristSubsystem* wrist) :
        wrist(wrist)
    {
        this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(wrist));
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

}  // namespace control

}  // namespace aruwsrc