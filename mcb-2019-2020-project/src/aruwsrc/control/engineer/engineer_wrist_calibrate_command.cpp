#include "engineer_wrist_calibrate_command.hpp"
#include "src/aruwlib/control/subsystem.hpp"

namespace aruwsrc
{

namespace control
{

    EngineerWristCalibrateCommand::EngineerWristCalibrateCommand(EngineerWristSubsystem* wrist) :
        wrist(wrist)
    {
        this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(wrist));
    }

    void EngineerWristCalibrateCommand::initialize()
    {
        wrist->wristCalibrateHere();
    }

    void EngineerWristCalibrateCommand::execute()
    {
        wrist->wristCalibrateHere();
    }

    void EngineerWristCalibrateCommand::end(bool interrupted)
    {}

    bool EngineerWristCalibrateCommand::isFinished() const
    {
        return wrist->wristCalibrateHere();
    }

    EngineerWristSubsystem* wrist;

}  // namespace control

}  // namespace aruwsrc