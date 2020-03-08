#ifndef __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__
#define __COMMAND_SENTINEL_DRIVE_RANDOM_HPP__

#include "src/aruwlib/control/command.hpp"
#include "sentinel_drive_subsystem.hpp"
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

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished() const;

    void interrupted();

 private:
    static const int16_t MIN_RPM = 5000;
    static const int16_t MAX_RPM = 7000;
    static const int16_t CHANGE_TIME_INTERVAL = 1000;
    static constexpr float RAIL_BUFFER = 0.1f * SentinelDriveSubsystem::RAIL_LENGTH;

    float currentRPM = 0;

    SentinelDriveSubsystem* subsystemSentinelDrive;
    modm::ShortTimeout changeVelocityTimer;
};

}  // namespace control

}  // namespace aruwsrc

#endif
