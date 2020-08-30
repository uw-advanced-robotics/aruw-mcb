#ifndef EXTEND_XAXIS_COMMAND_HPP_
#define EXTEND_XAXIS_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

#include "xaxis_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
/**
 * Call this command to extend the x axis on the engineer.
 * This sends a digital out signal to a solenoid, which actuates
 * a piston, used for collecting far bins.
 */
template <typename Drivers> class ExtendXAxisCommand : public aruwlib::control::Command<Drivers>
{
public:
    ExtendXAxisCommand(XAxisSubsystem<Drivers>* subsystem) : xAxisSubsystem(subsystem)
    {
        this->addSubsystemRequirement(subsystem);
    }

    void initialize() override
    {
        xAxisSubsystem->setExtended(true);  // default movement is "not extended"
    }

    void execute() override {}

    void end(bool) override { xAxisSubsystem->setExtended(false); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "extend x-axis command"; }

private:
    XAxisSubsystem<Drivers>* xAxisSubsystem;
};  // class ExtendXAxisCommand

}  // namespace engineer

}  // namespace aruwsrc

#endif  // EXTEND_XAXIS_COMMAND_HPP_
