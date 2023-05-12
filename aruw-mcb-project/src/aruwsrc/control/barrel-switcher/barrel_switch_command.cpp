#include "barrel_switch_command.hpp"

namespace aruwsrc::control {

    void BarrelSwitchCommand::execute() {
        if (barrelSwitcher.isBetweenPositions()) {
            finished = true;
            return;
        }
        if (barrelSwitcher.getBarrelState() == BarrelState::USING_LEFT_BARREL || 
            barrelSwitcher.getBarrelState() == BarrelState::IDLE) {
            barrelSwitcher.moveTowardUpperBound();
        } else if (barrelSwitcher.getBarrelState() == BarrelState::USING_RIGHT_BARREL) {
            barrelSwitcher.moveTowardLowerBound();
        }
        finished = true;
    }

    bool BarrelSwitchCommand::isFinished() const {
        return finished;
    }
} // namespace aruwsrc::control