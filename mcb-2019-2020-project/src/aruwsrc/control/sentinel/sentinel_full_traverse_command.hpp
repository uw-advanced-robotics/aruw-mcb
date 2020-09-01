#ifndef SENTINEL_FULL_TRAVERSE_COMMAND_HPP_
#define SENTINEL_FULL_TRAVERSE_COMMAND_HPP_

#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/architecture/clock.hpp>
#include <aruwlib/control/command.hpp>

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
class SentinelDriveSubsystem;

class SentinelFullTraverseCommand : public aruwlib::control::Command
{
public:
    explicit SentinelFullTraverseCommand(SentinelDriveSubsystem* subsystem);

    void initialize();

    void execute();

    void end(bool);

    bool isFinished() const;

private:
    /**
     * Rate of change of the sentinel when changing direction, in rpm of the sentinel's wheels per
     * ms
     */
    static constexpr float RAMP_SPEED = 10.0f;

    /**
     * The rotational speed of the sentinel's wheels before gearing is applied, in RPM.
     */
    static constexpr float MAX_DESIRED_TRAVERSE_SPEED = 5000.0f;

    /**
     * The distance from the end of the rail at which the sentinel will referse direction.
     */
    static constexpr float TURNAROUND_BUFFER = 0.3f * SentinelDriveSubsystem::RAIL_LENGTH;

    uint32_t prevTime;

    aruwlib::algorithms::Ramp velocityTargetGenerator;

    SentinelDriveSubsystem* subsystemSentinelDrive;
};  // class SentinelFullTraverseCommand

}  // namespace control

}  // namespace aruwsrc

#endif  // SENTINEL_FULL_TRAVERSE_COMMAND_HPP_
