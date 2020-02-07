#include <stdlib.h>

#include "sentinel_drive_random_command.hpp"
#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    SentinelDriveRandomCommand::SentinelDriveRandomCommand(SentinelDriveSubsystem* subsystem)
        : Command(), subsystemSentinelDrive(subsystem), sleepTimeout(CHANGE_TIME_INTERVAL)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
    }

    void SentinelDriveRandomCommand::initialize()
    {}

    void SentinelDriveRandomCommand::execute()
    {
        if (this->sleepTimeout.isExpired()) {
            this->sleepTimeout.restart(CHANGE_TIME_INTERVAL);
            currentRPM = rand() % MAX_RPM + 1;  // NOLINT
            if (currentRPM < MAX_RPM / 2) {
                currentRPM -= MAX_RPM;
            }
        }

        // reverse direction if close to the end of the rail
        float curPos = subsystemSentinelDrive->absolutePosition();
        if ((currentRPM < 0 && curPos < RAIL_BUFFER) ||
            (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH - RAIL_BUFFER)) {
            currentRPM = -currentRPM;
        }

        subsystemSentinelDrive->setDesiredRpm(currentRPM);
    }

    void SentinelDriveRandomCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            subsystemSentinelDrive->setDesiredRpm(0);
        }
        subsystemSentinelDrive->setDesiredRpm(0);
    }

    bool SentinelDriveRandomCommand::isFinished(void) const
    {
        return false;
    }

    void SentinelDriveRandomCommand::interrupted(void)
    {}
}  // namespace control

}  // namespace aruwsrc
