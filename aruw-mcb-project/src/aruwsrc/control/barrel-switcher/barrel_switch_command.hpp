
#ifndef BARREL_SWITCH_COMMAND_HPP_
#define BARREL_SWITCH_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/barrel-switcher/barrel_switcher_subsystem.hpp"

namespace aruwsrc::control
{

class BarrelSwitchCommand : public tap::control::Command
{
public:

    /**
     * The state of the command, whether the barrel switching is being controlled 
     * through manual input, or automatically using the HeatTrackers
    */
    enum class SwitchControlMode {
        MANUAL_CONTROL,
        AUTO_HEAT_CONTROL
    };

    //needs to take in two HeatTracker references once those are made
    BarrelSwitchCommand(aruwsrc::control::BarrelSwitcherSubsystem& barrelSwitcher)
        : barrelSwitcher(barrelSwitcher)
    {
        addSubsystemRequirement(&barrelSwitcher);
    };

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupt) override;

    const char* getName() const override { return "barrel switch command"; };
private:
    void switchBarrels();

    aruwsrc::control::BarrelSwitcherSubsystem& barrelSwitcher;
    SwitchControlMode controlMode;

}; // class BarrelSwitchCommand
} // namespace aruwsrc::control

#endif