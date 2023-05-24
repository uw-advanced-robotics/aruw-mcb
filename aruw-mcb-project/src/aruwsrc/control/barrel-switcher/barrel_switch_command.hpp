
#ifndef BARREL_SWITCH_COMMAND_HPP_
#define BARREL_SWITCH_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/barrel-switcher/barrel_switcher_subsystem.hpp"
#include "aruwsrc/control/governor/heat_tracker.hpp"

namespace aruwsrc::control
{

class BarrelSwitchCommand : public tap::control::Command
{
public:
    enum SwitchingControlState : uint8_t
    {
        USING_RIGHT = 0,
        USING_LEFT = 1,
        AUTOMATIC = 2,
        NUM_STATES = 3
    };

    BarrelSwitchCommand(
        aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher,
        tap::Drivers& drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID
            firingSystemMechanismIDLeft,
        const tap::communication::serial::RefSerialData::Rx::MechanismID
            firingSystemMechanismIDRight,
        const uint16_t heatLimitBuffer)
        : barrelSwitcher(barrelSwitcher),
          heatTrackerLeft(drivers, firingSystemMechanismIDLeft, heatLimitBuffer),
          heatTrackerRight(drivers, firingSystemMechanismIDRight, heatLimitBuffer)
    {
        addSubsystemRequirement(barrelSwitcher);
    };

    void initialize() override;

    void execute() override;

    void setControlState(SwitchingControlState state);

    bool isFinished() const override;

    void end(bool interrupt) override;

    const char* getName() const override { return "barrel switch command"; };

private:
    aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher;
    aruwsrc::control::governor::HeatTracker heatTrackerLeft;
    aruwsrc::control::governor::HeatTracker heatTrackerRight;
    BarrelSwitchCommand::SwitchingControlState controlState;
};  // class BarrelSwitchCommand
}  // namespace aruwsrc::control

#endif