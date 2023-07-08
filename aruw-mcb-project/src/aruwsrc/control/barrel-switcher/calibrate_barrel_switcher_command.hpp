#ifndef CALIBRATE_BARREL_SWITCHER_COMMAND_HPP_
#define CALIBRATE_BARREL_SWITCHER_COMMAND_HPP_

#include "tap/control/command.hpp"

namespace aruwsrc::control::barrel_switcher
{
class BarrelSwitcherSubsystem;

class CalibrateBarrelSwitcherCommand : public tap::control::Command
{
public:
    CalibrateBarrelSwitcherCommand(BarrelSwitcherSubsystem &barrelSwitcher);

    const char *getName() const override { return "calibrate barrel switcher"; }

    void initialize() override;

    void execute() override {}

    void end(bool) override {}

    bool isFinished() const override;

private:
    BarrelSwitcherSubsystem &barrelSwitcher;
};
}  // namespace aruwsrc::control::barrel_switcher

#endif  // CALIBRATE_BARREL_SWITCHER_COMMAND_HPP_
