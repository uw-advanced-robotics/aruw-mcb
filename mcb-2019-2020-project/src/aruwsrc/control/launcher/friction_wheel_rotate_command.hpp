#ifndef __FRICTION_WHEEL_ROTATE_COMMAND_HPP__
#define __FRICTION_WHEEL_ROTATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "friction_wheel_subsystem.hpp"

namespace aruwsrc
{
namespace launcher
{
template <typename Drivers> class FrictionWheelRotateCommand : public aruwlib::control::Command
{
public:
    FrictionWheelRotateCommand(FrictionWheelSubsystem<Drivers>* subsystem, int speed)
        : frictionWheelSubsystem(subsystem),
          speed(speed)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
    }

    void initialize() override {}

    void execute() override { frictionWheelSubsystem->setDesiredRpm(speed); }

    void end(bool) override { frictionWheelSubsystem->setDesiredRpm(0.0f); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "friction wheel rotate command"; }

    static const int16_t DEFAULT_WHEEL_RPM = 6000;

private:
    FrictionWheelSubsystem<Drivers>* frictionWheelSubsystem;

    int speed;
};

}  // namespace launcher

}  // namespace aruwsrc

#endif
