
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

    //needs to take in two HeatTracker references once those are made
    BarrelSwitchCommand(aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher)
        : barrelSwitcher(barrelSwitcher),
        finished(false)
    {
        addSubsystemRequirement(barrelSwitcher);
    };

    void initialize() override {};

    void execute() override;

    bool isFinished() const override;

    void end(bool) override {};

    const char* getName() const override { return "barrel switch command"; };
private:

    aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher;
    bool finished;
}; // class BarrelSwitchCommand
} // namespace aruwsrc::control

#endif