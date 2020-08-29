#ifndef __BLINK_LED_COMMAND_HPP__
#define __BLINK_LED_COMMAND_HPP__

#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>

#include "example_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
template <typename Drivers> class BlinkLEDCommand : public aruwlib::control::Command
{
public:
    explicit BlinkLEDCommand(aruwsrc::control::ExampleSubsystem<Drivers>* subsystem)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
    }

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    void initialize() override
    {
        completedTimer.restart(3000);
        startCounter++;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    void execute() override
    {
        refershCounter++;
        Drivers::leds.set(aruwlib::gpio::Leds::A, true);
    }

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    bool isFinished() const override { return completedTimer.isExpired(); }

    void end(bool) override { completedTimer.stop(); }

    const char* getName() const override { return "blink led command"; }

    aruwlib::arch::MilliTimeout completedTimer;

    int refershCounter = 0;
    int startCounter = 0;
};

}  // namespace control

}  // namespace aruwsrc

#endif
