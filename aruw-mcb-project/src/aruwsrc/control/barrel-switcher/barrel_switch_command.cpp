#include "barrel_switch_command.hpp"

namespace aruwsrc::control {

    void BarrelSwitchCommand::execute() {
        if (barrelSwitcher->getBarrelState() == BarrelState::USING_LEFT_BARREL || 
            barrelSwitcher->getBarrelState() == BarrelState::IDLE) {
            barrelSwitcher->useRight();
        } else if (barrelSwitcher->getBarrelState() == BarrelState::USING_RIGHT_BARREL) {
            barrelSwitcher->useLeft();
        }
        finished = true;
    }

    bool BarrelSwitchCommand::isFinished() const {
        return finished;
    }
} // namespace aruwsrc::control