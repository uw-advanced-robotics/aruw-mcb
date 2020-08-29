#ifndef SQUEEZE_GRABBER_COMMAND_HPP_
#define SQUEEZE_GRABBER_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

#include "grabber_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
template <typename Drivers> class SqueezeGrabberCommand : public aruwlib::control::Command
{
public:
    explicit SqueezeGrabberCommand(GrabberSubsystem<Drivers>* subsystem)
        : Command(),
          grabber(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(grabber));
    }
    void initialize() override { grabber->setSqueezed(true); }

    void execute() override {}

    void end(bool) override { grabber->setSqueezed(false); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "squeeze grabber command"; }

private:
    GrabberSubsystem<Drivers>* grabber;
};  // class SqueezeGrabberCommand

}  // namespace engineer

}  // namespace aruwsrc
#endif  // SQUEEZE_GRABBER_COMMAND_HPP_
