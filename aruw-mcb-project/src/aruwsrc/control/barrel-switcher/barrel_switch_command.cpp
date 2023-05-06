#include "barrel_switch_command.hpp"

namespace aruwsrc::control {

    void BarrelSwitchCommand::execute() {
        if (barrelSwitcher.getBarrelState() == BarrelState::USING_LEFT_BARREL) {
            barrelSwitcher.moveTowardUpperBound();
        } else {
            barrelSwitcher.moveTowardLowerBound();
        }
        finished = true;
    }

    bool BarrelSwitchCommand::isFinished() const {
        return finished;
    }
} // namespace aruwsrc::control