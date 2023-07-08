#include "calibrate_barrel_switcher_command.hpp"

#include "barrel_switcher_subsystem.hpp"

namespace aruwsrc::control::barrel_switcher
{
CalibrateBarrelSwitcherCommand::CalibrateBarrelSwitcherCommand(
    BarrelSwitcherSubsystem &barrelSwitcher)
    : barrelSwitcher(barrelSwitcher)
{
}

void CalibrateBarrelSwitcherCommand::initialize() { barrelSwitcher.requestCalibration(); }

bool CalibrateBarrelSwitcherCommand::isFinished() const
{
    return barrelSwitcher.getCurBarrelMechId().has_value();
}
}  // namespace aruwsrc::control::barrel_switcher
