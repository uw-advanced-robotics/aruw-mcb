
#ifndef BARREL_SWITCH_COMMAND_HPP_
#define BARREL_SWITCH_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/barrel-switcher/barrel_switcher_subsystem.hpp"
#include "aruwsrc/control/governor/heat_tracker.hpp"

namespace aruwsrc::control
{

enum class SwitchingControlState {
    USING_RIGHT,
    USING_LEFT,
    AUTOMATIC
};

class BarrelSwitchCommand : public tap::control::Command
{
public:

    //needs to take in two HeatTracker references once those are made
    BarrelSwitchCommand(aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher,
        tap::Drivers &drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismIDLeft,
        const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismIDRight,
        const uint16_t heatLimitBuffer)
        : heatTrackerLeft(drivers, firingSystemMechanismIDLeft, heatLimitBuffer),
        heatTrackerRight(drivers, firingSystemMechanismIDRight, heatLimitBuffer),
        barrelSwitcher(barrelSwitcher)
    {
        addSubsystemRequirement(barrelSwitcher);
    };

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override {};

    const char* getName() const override { return "barrel switch command"; };
private:

    aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher;
    aruwsrc::control::governor::HeatTracker heatTrackerLeft;
    aruwsrc::control::governor::HeatTracker heatTrackerRight;
    aruwsrc::control::SwitchingControlState controlState;
}; // class BarrelSwitchCommand
} // namespace aruwsrc::control

#endif