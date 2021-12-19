#include "rotate_waterwheel_command.hpp"

#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

namespace aruwsrc::agitator
{
RotateWaterwheelCommand::RotateWaterwheelCommand(
    aruwsrc::Drivers* drivers,
    AgitatorSubsystem* waterwheel,
    const KickerAgitatorSubsystem* kicker)
    : MoveUnjamComprisedCommand(
          drivers,
          waterwheel,
          WATERWHEEL_42MM_CHANGE_ANGLE,
          WATERWHEEL_42MM_MAX_UNJAM_ANGLE,
          WATERWHEEL_42MM_ROTATE_TIME,
          WATERWHEEL_42MM_PAUSE_AFTER_ROTATE_TIME),
      drivers(drivers),
      kicker(kicker)
{
}

bool RotateWaterwheelCommand::isReady()
{
    return MoveUnjamComprisedCommand::isReady() && !kicker->isProjectileQueued();
}

bool RotateWaterwheelCommand::isFinished() const
{
    return MoveUnjamComprisedCommand::isFinished() || kicker->isProjectileQueued();
}
}  // namespace aruwsrc::agitator
