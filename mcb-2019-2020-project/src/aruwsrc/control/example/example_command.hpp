/**
 * This code is part of aruw's repository
 *
 * Example code for a default command for the subsystem-example subsystem.
 */

#ifndef __COMMAND_EXAMPLE_HPP__
#define __COMMAND_EXAMPLE_HPP__

#include <aruwlib/control/command.hpp>

#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
template <typename Drivers> class ExampleCommand : public aruwlib::control::Command
{
public:
    explicit ExampleCommand(ExampleSubsystem<Drivers>* subsystem, int speed)
        : Command(),
          subsystemExample(subsystem),
          speed(speed)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
    }

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    void initialize() override {}

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    void execute() override { subsystemExample->setDesiredRpm(speed); }

    /**
     * The action to take when the command ends.  Called when either the command
     * finishes normally, or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    void end(bool interrupted) override { subsystemExample->setDesiredRpm(0); }

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    bool isFinished() const override { return false; }

    const char* getName() const override { return "example command"; }

    static constexpr int16_t DEFAULT_WHEEL_RPM = 6000;

private:
    ExampleSubsystem<Drivers>* subsystemExample;

    int speed;
};

}  // namespace control

}  // namespace aruwsrc
#endif
