#ifndef __SENTINEL_FULL_TRAVERSE_COMMAND_HPP__
#define __SENTINEL_FULL_TRAVERSE_COMMAND_HPP__

#include "src/aruwlib/algorithms/ramp.hpp"
#include "src/aruwlib/control/command.hpp"
#include "sentinel_drive_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class SentinelDriveSubsystem;

class SentinelFullTraverseCommand : public Command
{
 public:
    explicit SentinelFullTraverseCommand(SentinelDriveSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool);

    bool isFinished() const;

 private:
    // units: rpm/ms
    static constexpr float RAMP_SPEED = 10.0f;

    static constexpr float MAX_DESIRED_TRAVERSE_SPEED = 5000.0f;

    static constexpr float RAIL_BUFFER = 0.3f * SentinelDriveSubsystem::RAIL_LENGTH;

    uint32_t prevTime = 0;

    aruwlib::algorithms::Ramp velocityTargetGenerator;

    SentinelDriveSubsystem* subsystemSentinelDrive;
};

}  // namespace control

}  // namespace aruwsrc

#endif
