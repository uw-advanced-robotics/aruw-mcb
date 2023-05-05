#include "barrel_switch_command.hpp"

namespace aruwsrc::control {

    void BarrelSwitchCommand::initialize() {
        controlMode = SwitchControlMode::MANUAL_CONTROL;
    }

    void BarrelSwitchCommand::execute() {
        switch(controlMode) {
            case (SwitchControlMode::MANUAL_CONTROL):
            {
                
                break;
            }
            case (SwitchControlMode::AUTO_HEAT_CONTROL):
            {

                break;
            }
        }
    }

    bool BarrelSwitchCommand::isFinished() const {
        return false;
    }

    void BarrelSwitchCommand::end(bool interrupt) {
        barrelSwitcher.stop();
    }
} // namespace aruwsrc::control