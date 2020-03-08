#include "sentinel_drive_manual_command.hpp"

#include <stdlib.h>

#include "sentinel_drive_random_command.hpp"
#include "sentinel_drive_subsystem.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace control
{
    SentinelDriveManualCommand::SentinelDriveManualCommand(SentinelDriveSubsystem* subsystem)
        : Command(), subsystemSentinelDrive(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void SentinelDriveManualCommand::initialize()
    {}

    void SentinelDriveManualCommand::execute()
    {
        subsystemSentinelDrive->setDesiredRpm(getUserDesiredSentinelSpeed());
    }

    // NOLINTNEXTLINE
    void SentinelDriveManualCommand::end(bool)
    {
        subsystemSentinelDrive->setDesiredRpm(0);
    }

    bool SentinelDriveManualCommand::isFinished() const
    {
        return false;
    }

    void SentinelDriveManualCommand::interrupted()
    {}

    float SentinelDriveManualCommand::getUserDesiredSentinelSpeed()
    {
        return aruwlib::Remote::getChannel(aruwlib::Remote::Channel::LEFT_HORIZONTAL)
                * MAX_USER_DRIVE_SPEED;
    }
}  // namespace control

}  // namespace aruwsrc
