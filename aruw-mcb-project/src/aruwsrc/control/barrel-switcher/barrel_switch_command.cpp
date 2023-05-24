#include "barrel_switch_command.hpp"

namespace aruwsrc::control
{

void BarrelSwitchCommand::initialize() { controlState = SwitchingControlState::AUTOMATIC; }

void BarrelSwitchCommand::execute()
{
    switch (controlState)
    {
        case SwitchingControlState::USING_LEFT:
            if (barrelSwitcher->getBarrelState() != BarrelState::USING_LEFT_BARREL)
            {
                barrelSwitcher->useLeft();
            }
            break;
        case SwitchingControlState::USING_RIGHT:
            if (barrelSwitcher->getBarrelState() != BarrelState::USING_RIGHT_BARREL)
            {
                barrelSwitcher->useRight();
            }
            break;
        case SwitchingControlState::AUTOMATIC:
            if (barrelSwitcher->getBarrelState() == BarrelState::IDLE)
            {
                barrelSwitcher->useRight();
            }
            if (barrelSwitcher->getBarrelState() == BarrelState::USING_LEFT_BARREL &&
                !heatTrackerLeft.enoughHeatToLaunchProjectile())
            {
                barrelSwitcher->useRight();
            }
            else if (
                barrelSwitcher->getBarrelState() == BarrelState::USING_RIGHT_BARREL &&
                !heatTrackerRight.enoughHeatToLaunchProjectile())
            {
                barrelSwitcher->useLeft();
            }
            break;
        case SwitchingControlState::NUM_STATES:
            break;
    }
}

void BarrelSwitchCommand::setControlState(SwitchingControlState state)
{
    if (state < SwitchingControlState::NUM_STATES)
    {
        this->controlState = state;
    }
}

void BarrelSwitchCommand::end(bool) { barrelSwitcher->stop(); }

bool BarrelSwitchCommand::isFinished() const { return false; }
}  // namespace aruwsrc::control