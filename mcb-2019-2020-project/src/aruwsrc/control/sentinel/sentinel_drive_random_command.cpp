#include <stdlib.h>

#include "sentinel_drive_random_command.hpp"
#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    SentinelDriveRandomCommand::SentinelDriveRandomCommand(SentinelDriveSubsystem* subsystem)
        : Command(), subsystemSentinelDrive(subsystem), changeVelocityTimer(CHANGE_TIME_INTERVAL)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void SentinelDriveRandomCommand::initialize()
    {}

    void SentinelDriveRandomCommand::execute()
    {
        if (this->changeVelocityTimer.isExpired()) {
            this->changeVelocityTimer.restart(CHANGE_TIME_INTERVAL);
            // NOLINTNEXTLINE
            currentRPM = rand() % (MAX_RPM - MIN_RPM + 1) + MIN_RPM;
            // NOLINTNEXTLINE
            if (rand() % 2 == 0) {
                currentRPM *= -1.0f;
            }
        }

        // reverse direction if close to the end of the rail
        float curPos = subsystemSentinelDrive->absolutePosition();
        if ((currentRPM < 0 && curPos < RAIL_BUFFER + SentinelDriveSubsystem::SENTINEL_WIDTH / 2.0f) ||
            (currentRPM > 0 && curPos > SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_WIDTH / 2.0f - RAIL_BUFFER)) {
            currentRPM = -currentRPM;
        }

        subsystemSentinelDrive->setDesiredRpm(currentRPM);
    }

    // NOLINTNEXTLINE
    void SentinelDriveRandomCommand::end(bool)
    {
        subsystemSentinelDrive->setDesiredRpm(0);
    }

    bool SentinelDriveRandomCommand::isFinished() const
    {
        return false;
    }

    void SentinelDriveRandomCommand::interrupted()
    {}
}  // namespace control

}  // namespace aruwsrc
