#include "barrel_switch_command.hpp"

namespace aruwsrc::control {

    void BarrelSwitchCommand::execute() {
        switch (controlState) {
            case SwitchingControlState::USING_LEFT:
                if (barrelSwitcher->getBarrelState() != BarrelState::USING_LEFT_BARREL){
                    barrelSwitcher->useLeft();
                }
                break;
            case SwitchingControlState::USING_RIGHT:
                if (barrelSwitcher->getBarrelState() != BarrelState::USING_RIGHT_BARREL){
                    barrelSwitcher->useRight();
                }
                break;
            case SwitchingControlState::AUTOMATIC:
                //if overheat:
                    if (barrelSwitcher->getBarrelState() == BarrelState::USING_LEFT_BARREL || 
                        barrelSwitcher->getBarrelState() == BarrelState::IDLE) {
                        barrelSwitcher->useRight();
                    } else if (barrelSwitcher->getBarrelState() == BarrelState::USING_RIGHT_BARREL) {
                        barrelSwitcher->useLeft();
                    }
                break;
        }
    }

    bool BarrelSwitchCommand::isFinished() const {
        return finished;
    }
} // namespace aruwsrc::control