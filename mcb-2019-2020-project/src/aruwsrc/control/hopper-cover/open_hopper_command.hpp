#ifndef __OPEN_HOPPER_COMMAND_HPP__
#define __OPEN_HOPPER_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "hopper_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
template <typename Drivers> class OpenHopperCommand : public aruwlib::control::Command<Drivers>
{
public:
    explicit OpenHopperCommand(HopperSubsystem<Drivers>* subsystem) : subsystemHopper(subsystem)
    {
        this->addSubsystemRequirement(subsystem);
    }

    void initialize() override { subsystemHopper->setOpen(); }

    void execute() override {}

    void end(bool) override { subsystemHopper->setClose(); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "open hopper command"; }

private:
    HopperSubsystem<Drivers>* subsystemHopper;
};

}  // namespace control

}  // namespace aruwsrc
#endif
