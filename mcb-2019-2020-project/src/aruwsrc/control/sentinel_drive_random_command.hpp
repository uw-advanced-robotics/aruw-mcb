#ifndef __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__
#define __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__

#include "src/aruwlib/control/command.hpp"
#include "src/aruwsrc/control/sentinel_drive_subsystem.hpp"
#include "modm/processing/timer.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class SentinelDriveSubsystem;

class SentinelDriveRandomCommand : public Command
{
 public:
    explicit SentinelDriveRandomCommand(SentinelDriveSubsystem* subsystem = nullptr);

    /**
      * The initial subroutine of a command.  Called once when the command is
      * initially scheduled.
      */
    void initialize(void);

    /**
      * The main body of a command.  Called repeatedly while the command is
      * scheduled.
      */
    void execute(void);

    /**
      * The action to take when the command ends.  Called when either the command
      * finishes normally, or when it interrupted/canceled.
      *
      * @param interrupted whether the command was interrupted/canceled
      */
    void end(bool interrupted);

    /**
      * Whether the command has finished.  Once a command finishes, the scheduler
      * will call its end() method and un-schedule it.
      *
      * @return whether the command has finished.
      */
    bool isFinished(void) const;

    void interrupted(void);

 private:
    static constexpr int16_t MAX_RPM = 4000;
    static constexpr float RAIL_BUFFER = 0.1 * SentinelDriveSubsystem::RAIL_LENGTH;
    static const int16_t CHANGE_TIME_INTERVAL = 5000;

    float currentRPM = 0;

    SentinelDriveSubsystem* subsystemSentinelDrive;
    modm::ShortTimeout sleepTimeout;
};

}  // namespace control

}  // namespace aruwsrc
#endif
